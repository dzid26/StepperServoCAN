 /**
 * MKS SERVO42B
 * Copyright (c) 2020 Makerbase.
 *
 * Based on nano_stepper project by Misfittech
 * Copyright (C) 2018  MisfitTech LLC.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "stepper_controller.h"

extern volatile bool forwardRotation;
extern volatile bool A4950_Enabled;
extern volatile uint8_t DIR;
extern volatile uint32_t NVM_address;
extern nvm_t nvmParams;

volatile MotorParams_t motorParams;
volatile SystemParams_t systemParams;
volatile PID_t sPID; //simple control loop PID parameters
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

volatile int32_t loopError = 0;
volatile bool StepperCtrl_Enabled = true;
volatile bool TC1_ISR_Enabled = false;
volatile bool enableFeedback = false; //true if we are using PID control algorithm
volatile int32_t angleFullStep = 327;

volatile int32_t zeroAngleOffset = 0;
volatile int32_t desiredLocation;
volatile int32_t currentLocation = 0;
volatile int32_t speed_slow = 0;

void setupTCInterrupts(void)
{
	RCC_ClocksTypeDef clocksData;
	RCC_GetClocksFreq(&clocksData);

	TIM_DeInit(TIM1);
	TIM_TimeBaseInitTypeDef  		TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = (clocksData.PCLK2_Frequency / MHz_to_Hz -1);	//Prescale to 1MHz
	TIM_TimeBaseStructure.TIM_Period = SAMPLING_PERIOD_uS-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //has to be zero to not skip period ticks
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_SetCounter(TIM1, 0);
	TIM_Cmd(TIM1, ENABLE);
}

void enableTCInterrupts(void)
{
	TC1_ISR_Enabled = true;
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

void disableTCInterrupts(void)
{
	TC1_ISR_Enabled = false;
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

void StepperCtrl_updateParamsFromNVM(void)
{
	if(NVM->SystemParams.parametersValid == valid)
	{
		pPID.Kp = NVM->pPID.Kp * CTRL_PID_SCALING;
		pPID.Ki = NVM->pPID.Ki * CTRL_PID_SCALING;
		pPID.Kd = NVM->pPID.Kd * CTRL_PID_SCALING;

		vPID.Kp = NVM->vPID.Kp * CTRL_PID_SCALING;
		vPID.Ki = NVM->vPID.Ki * CTRL_PID_SCALING;
		vPID.Kd = NVM->vPID.Kd * CTRL_PID_SCALING;

		sPID.Kp = NVM->sPID.Kp * CTRL_PID_SCALING;
		sPID.Ki = NVM->sPID.Ki * CTRL_PID_SCALING;
		sPID.Kd = NVM->sPID.Kd * CTRL_PID_SCALING;

		systemParams = NVM->SystemParams;
	}else
	{
		pPID.Kp = nvmParams.pPID.Kp * CTRL_PID_SCALING;
		pPID.Ki = nvmParams.pPID.Ki * CTRL_PID_SCALING;
		pPID.Kd = nvmParams.pPID.Kd * CTRL_PID_SCALING;

		vPID.Kp = nvmParams.vPID.Kp * CTRL_PID_SCALING;
		vPID.Ki = nvmParams.vPID.Ki * CTRL_PID_SCALING;
		vPID.Kd = nvmParams.vPID.Kd * CTRL_PID_SCALING;

		sPID.Kp = nvmParams.sPID.Kp * CTRL_PID_SCALING;
		sPID.Ki = nvmParams.sPID.Ki * CTRL_PID_SCALING;
		sPID.Kd = nvmParams.sPID.Kd * CTRL_PID_SCALING;

		systemParams = nvmParams.SystemParams;
	}

	//default the error pin to input, if it is an error pin the
	// handler for this will change the pin to be an output.
	// for bidirection error it has to handle input/output it's self as well.
	// This is not the cleanest way to handle this...
	// TODO implement this cleaner
	if(NVM->motorParams.parametersValid == valid)
	{
		motorParams = NVM->motorParams;
	} else
	{
		motorParams.fullStepsPerRotation = 200;

#ifndef MKS_SERVO42B
		motorParams.currentHoldMa = 800;
		motorParams.currentMa = 1440;
#else
		motorParams.currentHoldMa = 400;
		motorParams.currentMa = 800;
#endif

		motorParams.motorWiring = true;
	}

	StepperCtrl_setRotationDirection(motorParams.motorWiring);
}

void StepperCtrl_motorReset(void)
{
	//when we reset the motor we want to also sync the motor
	//phase.Therefore we move forward a few full steps then back
	//to sync motor phasing, leaving the motor at "phase 0"
	A4950_move(0,motorParams.currentMa);
	delay_ms(1200);

  //measure new starting point
	StepperCtrl_setLocationFromEncoder();
}

void StepperCtrl_setLocationFromEncoder(void)
{
	currentLocation = 0;

	if (CalibrationTable_calValid())
	{
		uint16_t x,a;

		//set our angles based on previous cal data
		x = StepperCtrl_sampleMeanEncoder(102);
		a = CalibrationTable_fastReverseLookup(x); //start angle

		currentLocation = (int32_t)a; //save position
	}
	zeroAngleOffset = StepperCtrl_updateCurrentLocation(); //zero the angle shown on LCD
	desiredLocation = zeroAngleOffset;
}

//estimate of the current location from encoder feedback
//the current location lower 16 bits is angle (0-360 degrees in 65536 steps) while upper
//bits is the number of full rotations.
int32_t StepperCtrl_updateCurrentLocation(void)
{
	int32_t a,x;

	a = (int32_t)StepperCtrl_getEncoderAngle();

	x = a - (currentLocation & (int32_t) ANGLE_MAX);

	if ( x > ANGLE_WRAP )
	{
		currentLocation -= (int32_t) ANGLE_STEPS;
	}
	if ( x < -ANGLE_WRAP )
	{
		currentLocation += (int32_t)  ANGLE_STEPS;
	}

	currentLocation = (currentLocation & 0xFFFF0000) | a;

	return currentLocation;
}

int32_t StepperCtrl_getDesiredLocation(void) //get angle
{
	int32_t ret;
	disableTCInterrupts(); 
	ret = desiredLocation;
	enableTCInterrupts();
	return ret;
}

int32_t StepperCtrl_getCurrentLocation(){
	int32_t ret;
	disableTCInterrupts(); 
	ret = currentLocation;
	enableTCInterrupts();
	return ret;
}

//The encoder needs to be calibrated to the motor.
// we will assume full step detents are correct,
// ex 1.8 degree motor will have 200 steps for 360 degrees.
// We also need to calibrate the phasing of the motor
// to the A4950. This requires that the A4950 "step angle" of
// zero is the first entry in the calibration table.
uint16_t StepperCtrl_calibrateEncoder(bool updateFlash)
{
	
	uint16_t maxError;
	int32_t microSteps = 0;
	uint8_t passes = 0;
	bool feedback = enableFeedback;
	bool state = TC1_ISR_Enabled;
	disableTCInterrupts();

	A4950_Enabled = true;
	enableFeedback = false;

	// Enable hardware averaging and disable hysteresis filter:
	// 	orate | hysteresis | zero_offset | ro | rd
	A1333_setRegister_ANG(12, 0, 0, 0, 0); 
	//max averaging of 12 is 4096 samples which is equates to 4ms lag. Assuming motor stay in the full step for at least 4ms the signal will be settled and fully filtered 

	A4950_move(0, motorParams.currentMa);
	delay_ms(50);
	uint16_t calZeroOffset=(StepperCtrl_sampleMeanEncoder(202)<<1) - (CalibrationTable_getCal(0)<<1);//returns (0-32767) scaled to (0-65535)
	maxError = CalibrationMove(updateFlash, 1, &microSteps, &passes, calZeroOffset); //run clockwise
	if(updateFlash) {	//second pass anticlockwise when calibrating
		calZeroOffset=0; //on the second run, the calTab values are alligned with the sensor, so no offset
		maxError = CalibrationMove(updateFlash, -1, &microSteps, &passes, calZeroOffset); 

		CalibrationTable_saveToFlash(); //saves the calibration to flash
		StepperCtrl_updateParamsFromNVM(); //update the local cache from the NVM
	}
	StepperCtrl_motorReset();
	A1333_begin(); //Reset filters and perform sensor tests

	enableFeedback = feedback;
	if (state) enableTCInterrupts();

	return maxError;
}

static uint16_t CalibrationMove(bool updateFlash, int8_t dir, int32_t *microSteps, uint8_t *passes, uint16_t calZeroOffset){
	
	uint16_t maxError = 0;
	
	(*passes)++;
	int16_t microStep = A4950_STEP_MICROSTEPS*motorParams.fullStepsPerRotation/CALIBRATION_TABLE_SIZE;
	
	const uint16_t preRunPart = CALIBRATION_TABLE_SIZE/2; //do half rotation preRun to start calibration with max hysteresis
	for (uint16_t j = 0; j < preRunPart + CALIBRATION_TABLE_SIZE; j++) //Starting calibration 
	{
		static volatile uint16_t desiredAngle;
		static volatile uint16_t sampled;

		bool preRun = j < preRunPart;
		delay_ms(1);
		if (updateFlash && !preRun) 
			delay_ms(50);

		
		desiredAngle = (uint16_t) DIVIDE_WITH_ROUND(*microSteps * (int32_t)(ANGLE_STEPS>>2) / A4950_STEP_MICROSTEPS, motorParams.fullStepsPerRotation>>2);
		uint16_t cal = (CalibrationTable_getCal(desiredAngle)<<1) + calZeroOffset; //returns (0-32767) scaled to (0-65535)
		
		sampled = StepperCtrl_sampleMeanEncoder(202)<<1; //collect angle every half step for 1.8 stepper - returns (0-32767) scaled to (0-65535)
		
		int16_t dist = (int16_t)(sampled - cal);

		
		if (updateFlash && !preRun) {
			uint16_t average = cal + (dist / (*passes)); //add half a distance to average
			CalibrationTable_updateTableValue(( (dir<0) ? (preRunPart + CALIBRATION_TABLE_SIZE -j) : j)%CALIBRATION_TABLE_SIZE, average>>1);	// (0-65535) scaled to (0-32767)
		}
		//move one half step at a time, a full step move could cause a move backwards
		
		dist = fastAbs(dist>>1);
		if(dist > maxError && !preRun)
				{
					maxError = dist;
				}
		if(microStep > A4950_STEP_MICROSTEPS)  //for 0.9deg stepper collect data only every full step
		{
			A4950_move(*microSteps + A4950_STEP_MICROSTEPS, motorParams.currentMa);
			delay_ms(20);
		}
		*microSteps += microStep*dir;
		A4950_move(*microSteps, motorParams.currentMa);
	}
	return maxError;
}

// when sampling the mean of encoder if we are on roll over
// edge we can have an issue so we have this function
// to do the mean correctly
uint16_t StepperCtrl_sampleMeanEncoder(uint16_t numSamples)
{
	uint16_t i;
	int32_t lastx = 0,x = 0;
	int32_t min = 0,max = 0;
	int64_t sum = 0;
	int32_t mean = 0;

	for(i=0; i < numSamples; i++)
	{
		lastx = x;
		x = (int32_t)A1333_readEncoderAngle();

		if(i == 0)
		{
			lastx = x;
			min = x;
			max = x;
		}

		//wrap
		if (fastAbs(lastx - x) > CALIBRATION_WRAP) //2^15-1 = 32767(max)
		{
			if (lastx > x)
			{
				x = x + CALIBRATION_STEPS;
			} else
			{
				x = x - CALIBRATION_STEPS;
			}
		}

		if (x > max)
		{
			max = x;
		}
		if (x < min)
		{
			min = x;
		}

		sum = sum + (x);
	}

	mean = (int32_t)(sum - min - max) / (numSamples - 2); //remove the min and the max.

	//mean 0~32767
	if(mean >= CALIBRATION_STEPS)
	{
		mean = (mean - (int32_t)CALIBRATION_STEPS);
	}
	if(mean < 0)
	{
		mean = (mean + (int32_t)CALIBRATION_STEPS);
	}

	return ((uint16_t)mean); //(0-32767)
}

uint16_t StepperCtrl_getEncoderAngle(void)
{
	uint16_t EncoderAngle;

	EncoderAngle = CalibrationTable_fastReverseLookup(A1333_readEncoderAngle()); //0-65535

	return EncoderAngle;
}

void StepperCtrl_setRotationDirection(bool forward)
{
	forwardRotation = forward;
}

// TODO This function does two things, set rotation direction
// and measures step size, it should be two functions.
// return is anlge in degreesx100 ie 360.0 is returned as 36000
float StepperCtrl_measureStepSize(void)
{
	int32_t angle1,angle2,x;
	bool feedback = enableFeedback;
	uint16_t microsteps = systemParams.microsteps;

	A4950_Enabled = true;
	systemParams.microsteps = 1;
	enableFeedback = false;
	motorParams.motorWiring = true; //assume we are forward wiring to start with
	StepperCtrl_setRotationDirection(motorParams.motorWiring);
	/////////////////////////////////////////
	//// Measure the full step size /////
	/// Note we assume machine can take one step without issue///

	A4950_move(0,motorParams.currentMa); //this puts stepper motor at stepAngle of zero
	delay_ms(1200);

	angle1 = StepperCtrl_sampleMeanEncoder(102); //angle1
	A4950_move(A4950_STEP_MICROSTEPS/2,motorParams.currentMa); //move one half step 'forward'
	delay_ms(100);
	A4950_move(A4950_STEP_MICROSTEPS,motorParams.currentMa); //move one half step 'forward'
	delay_ms(500);
	angle2 = StepperCtrl_sampleMeanEncoder(102); //angle2

	if ( fastAbs(angle2 - angle1) > CALIBRATION_WRAP )
	{
		//we crossed the wrap around
		if (angle1 > angle2)
		{
			angle1 = angle1 + (int32_t)CALIBRATION_STEPS;
		}else
		{
			angle2 = angle2 + (int32_t)CALIBRATION_STEPS;
		}
	}

	// if x is ~180 we have a 1.8 degree step motor, if it is ~90 we have 0.9 degree step
	x = (int32_t)(((int64_t)(angle2 - angle1) * 36000) / (int32_t)CALIBRATION_STEPS);

	//move back
	A4950_move(A4950_STEP_MICROSTEPS/2,motorParams.currentMa);
	delay_ms(100);
	A4950_move(0,motorParams.currentMa);

	systemParams.microsteps = microsteps;
	enableFeedback = feedback;

	return ((float)x)/100.0;
}

stepCtrlError_t StepperCtrl_begin(void)
{
	float x;
	enableFeedback = false;
	currentLocation = 0;

	//we have to update from NVM before moving motor
	StepperCtrl_updateParamsFromNVM(); //update the local cache from the NVM

	//start up encoder
	if (false == A1333_begin())
	{
		return STEPCTRL_NO_ENCODER;
	}

	//cal table init
	CalibrationTable_init();

//	A4950_begin();

	if (NVM->motorParams.parametersValid == valid)
	{
		//lets read the motor voltage
		if (0/*GetMotorVoltage()<5*/)
		{
			//if we have less than 5 volts the motor is not powered
//			uint32_t x;
//			x=(uint32_t)(GetMotorVoltage()*1000.0);
			return STEPCTRL_NO_POWER;
		}
	}else
	{
		x = StepperCtrl_measureStepSize();
		if (fabs(x) < 0.5)
		{
			return STEPCTRL_NO_POWER; //Motor may not have power
		}
	}

	//Checking the motor parameters
	//todo we might want to move this up a level to the NZS
	//especially since it has default values
	if (NVM->motorParams.parametersValid != valid) //NVM motor parameters are not set, we will update
	{
		// power could have just been applied and step size read wrong
		// if we are more than 200 steps/rotation which is most common
		// lets read again just to be sure.
		if (fabs(x) < 1.5)
		{
			//run step test a second time to be sure
			x = StepperCtrl_measureStepSize();
		}

		if (x > 0)
		{
			motorParams.motorWiring = true;
		} else
		{
			motorParams.motorWiring = false;
		}
		if (fabs(x) <= 1.2)
		{
			motorParams.fullStepsPerRotation = 400;
		}else
		{
			motorParams.fullStepsPerRotation = 200;
		}
		//Motor params are now good
		nvmParams.motorParams = motorParams;
		nvmWriteConfParms(&nvmParams);
	}

	angleFullStep = (int32_t)(ANGLE_STEPS / motorParams.fullStepsPerRotation);

	StepperCtrl_setLocationFromEncoder(); //measure new starting point

	if (false == CalibrationTable_calValid())
	{
		return STEPCTRL_NO_CAL;
	}

	enableFeedback = true;
	setupTCInterrupts();
	enableTCInterrupts();

	return STEPCTRL_NO_ERROR;
}

void StepperCtrl_enable(bool enable)
{
	A4950_enable(enable); //enable or disable the stepper driver as needed

	if(StepperCtrl_Enabled == true && enable == false)
	{
		enableFeedback = false;
		disableTCInterrupts();
	}
	if(StepperCtrl_Enabled == false && enable == true) //if we are enabling previous disabled motor
	{
		StepperCtrl_setLocationFromEncoder();
		enableFeedback = true;
		enableTCInterrupts();
	}

	StepperCtrl_Enabled = enable;
}

void StepperCtrl_updateDesiredLocation(int32_t deltaLocation){
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	desiredLocation = StepperCtrl_getCurrentLocation() + deltaLocation;
	enableTCInterrupts();
}

bool StepperCtrl_processFeedback(void)
{
	bool ret = false;
	int32_t desiredLoc;
	int32_t currentLoc;
	static int32_t lastLoc;
	const int16_t speed_filter_tc = 128; //speed filter time constant
	const int64_t error_filter_tc = 2; //error filter time constant - choose depending on CAN RX rate
	int32_t speed_raw;
	int32_t error;
	static int32_t desiredLoc_slow = 0;
	desiredLoc = StepperCtrl_getDesiredLocation(); //DesiredLocation
	currentLoc = StepperCtrl_updateCurrentLocation(); //CurrentLocation

	desiredLoc_slow = (desiredLoc + (error_filter_tc-1) * desiredLoc_slow) / error_filter_tc; 
	error = desiredLoc_slow - currentLoc; //error is desired - PoscurrentPos

	speed_raw = (currentLoc - lastLoc) * (int32_t) SAMPLING_HZ; // 360deg/65536/s
	speed_slow = (speed_raw + (speed_filter_tc-1) * speed_slow) / speed_filter_tc; 

	ret = StepperCtrl_simpleFeedback(error);

//	switch (systemParams.controllerMode)
//	{
//		case CTRL_POS_PID:
//		{
////			ret = StepperCtrl_pidFeedback(desiredLoc, currentLoc, &ctrl);
//			break;
//		}
//		default:
//		case CTRL_SIMPLE:
//		{
//			ret = StepperCtrl_simpleFeedback(desiredLoc, currentLoc);
//			break;
//		}
//		case CTRL_POS_VELOCITY_PID:
//		{
////			ret = StepperCtrl_vpidFeedback(desiredLoc, currentLoc, &ctrl);
//			break;
//		}
//	}
	lastLoc = currentLoc;
	return ret;
}

//this was written to do the PID loop not modeling a DC servo
// but rather using features of stepper motor.
bool StepperCtrl_simpleFeedback(int32_t error)
{
	static int32_t lastError = 0;
	static uint32_t errorCount = 0;
	const int16_t zerocrossMax = 20; //mA
	const uint16_t smallLoad = 25; //abs mA
	if(enableFeedback)
	{
		int32_t errorSat;
		int_fast16_t pTerm;
		int_fast16_t dTerm;
		static float iTerm = 0; //iTerm memory
		static uint8_t saturationId = 2; //0 is negative saturation, 1 is positive saturation, 2 is no saturation
		int_fast16_t feedForward;
		int_fast16_t closeLoop; 
		int_fast16_t closeLoopMax;
		int16_t loadAngleDesired;
		static uint16_t magnitude; //static for dTerm condition check
		
		closeLoopMax = (int32_t) motorParams.currentMa;
		feedForward  = (int16_t) motorParams.currentHoldMa;

		//protect pTern and iTerm against overflow
		int32_t errorMax = INT16_MAX * CTRL_PID_SCALING / max(sPID.Kp, sPID.Ki); 
		if( error > errorMax){
			errorSat = closeLoopMax;
		}else if (error < -errorMax)
		{
			errorSat = -closeLoopMax;
		}else{
			errorSat = error;
		}

		// PID proportional term
		pTerm  =  errorSat * sPID.Kp / CTRL_PID_SCALING;

		// PID derivative term
		if(magnitude > smallLoad){ //dTerm causes noise when motor is not loaded enough
			dTerm = (errorSat - lastError) * sPID.Kd / CTRL_PID_SCALING; // (error - lastError) is small
		}else{
			dTerm=0;
		}
		// PID integral term
		if((errorSat>0) != saturationId){ //antiwindup clamp - condition optimized using clever id values
			iTerm += (float) errorSat * sPID.Ki / CTRL_PID_SCALING / (int32_t) SAMPLING_PERIOD_uS;
		}
		
		closeLoop = pTerm + (int16_t) iTerm + dTerm;

		// If closeLoop is in the same direction as feedforward, closelooop is limited by closeLoopMax. 
		// If closeLoop is in opposite direction to feedforward, then it is limite to -feedforwad so that closeloop
		// has always power to cancel out the feedfrward  to avoid uncontrolled rotation
		if( closeLoop > max(closeLoopMax, -feedForward)) 
		{	
			closeLoop = max(closeLoopMax, -feedForward);
			saturationId = 1;
		} else if (closeLoop < -(max(closeLoopMax, feedForward)))
		{
			closeLoop = -(max(closeLoopMax, feedForward));
			saturationId = 0;
		} else {
			saturationId = 2;
		}
		
		closeLoop += feedForward;
		magnitude = (uint16_t) (fastAbs(closeLoop));
		
		//handle torque zero-crossing, otherwise request 90 elctrical degress torque vector
		if( closeLoop > zerocrossMax) 
		{	
			loadAngleDesired = angleFullStep;
		}
		else if (closeLoop < -zerocrossMax)
		{
			loadAngleDesired = -angleFullStep;
		}else{
			loadAngleDesired = 0; //closeLoop * angleFullStep / zerocrossMax; //scale or just set to 0 if it keeps rocking
		}

		int16_t loadAngleSpeedComp;//Compensate for angle sensor delay
		uint16_t angleSensLatency = (SAMPLING_PERIOD_uS + 80u);
		loadAngleSpeedComp = loadAngleDesired + (int16_t) (speed_slow * (int_fast16_t) angleSensLatency / (int32_t) S_to_uS  ); 
		StepperCtrl_moveToAngle(loadAngleSpeedComp, magnitude);
	
		lastError = errorSat;
		loopError = error;
	} //end if enableFeedback

  // error needs to exist for some time period
	if (abs(lastError) > systemParams.errorLimit)
	{
		++errorCount;
		if (errorCount > (SAMPLING_HZ >> 7))
		{
			return true;
		}
		return false;
	}

	if(errorCount > 0)
	{
		--errorCount;
	}

	return false;
}

void StepperCtrl_moveToAngle(int32_t loadAngle, uint16_t magnitude) 
{
	//we need to convert 'Angle' to A4950 steps
	loadAngle = StepperCtrl_getCurrentLocation() + loadAngle & ANGLE_MAX;	//we only interested in the rotor vs stator angle, which repeats after a half a rotation for 0.9stepper and full rotation for 1.8stepper 

	loadAngle = DIVIDE_WITH_ROUND(loadAngle * (motorParams.fullStepsPerRotation >> 3), ANGLE_STEPS / A4950_STEP_MICROSTEPS >> 3); //2^2=8 which is a common denominator of 200 and 256
	A4950_move(loadAngle, magnitude);
}