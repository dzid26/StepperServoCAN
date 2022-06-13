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

extern volatile bool A4950_Enabled;
extern volatile uint8_t DIR;
extern volatile uint32_t NVM_address;
extern nvm_t nvmParams;

volatile MotorParams_t motorParams;
volatile SystemParams_t systemParams;
volatile PID_t sPID; //simple control loop PID parameters
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

volatile bool StepperCtrl_Enabled = false;
extern volatile bool TC1_ISR_Enabled;
volatile bool enableFeedback = false; //true if control uses feedback
volatile bool enableCloseLoop = false; //true if control uses PID
volatile bool enableSoftOff = false; //true if soft off is enabled
volatile int32_t angleFullStep = 327;

volatile int32_t zeroAngleOffset = 0;

//api - commanded
volatile int32_t desiredLocation;
volatile int_fast16_t feedForward;
volatile int_fast16_t closeLoopMax;

//api - measured
volatile int32_t currentLocation = 0;
volatile int_fast16_t closeLoop;
volatile int_fast16_t control;
volatile int16_t Iq_ma;
volatile int32_t speed_slow = 0;
volatile int32_t loopError = 0;


uint16_t ReadEncoderAngle(void){ //15bits -  32767 is 360
	#ifdef MKS
		return A1333_readEncoderAngle();
	#elif BTT
		return TLE5012_ReadAngle();
		// uint16_t angle = 0;
		// errorTypes error = readAngleValue(&angle);
		// return angle;
	#else
		return 0;	
	#endif
}

bool Encoder_begin(void){
	#ifdef MKS
		return A1333_begin();
	#elif BTT
		return TLE5012_begin();
		// return true;
	#else
		return false;	
	#endif
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
	disableTCInterrupts();

	A4950_Enabled = true;
	enableFeedback = false;

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
	Encoder_begin(); //Reset filters and perform sensor tests

	enableFeedback = feedback;
	if (state) {
		enableTCInterrupts();
	}

	return maxError;
}

uint16_t CalibrationMove(bool updateFlash, int8_t dir, int32_t *microSteps, uint8_t *passes, uint16_t calZeroOffset){
	
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
			delay_ms(250);

		
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
		x = (int32_t)ReadEncoderAngle();
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

	EncoderAngle = CalibrationTable_fastReverseLookup(ReadEncoderAngle()); //0-65535

	return EncoderAngle;
}

// TODO This function does two things, set rotation direction
// return is anlge in degreesx100 ie 360.0 is returned as 36000
float StepperCtrl_measureStepSize(void)
{
	int32_t angle1,angle2,x;
	bool feedback = enableFeedback;
	enableFeedback = false;
	A4950_enable(true);

	/////////////////////////////////////////
	//// Measure the full step size /////
	/// Note we assume machine can take one step without issue///

	A4950_move(0,motorParams.currentMa); //this puts stepper motor at stepAngle of zero
	delay_ms(200);

	angle1 = StepperCtrl_sampleMeanEncoder(102); //angle1
	A4950_move(A4950_STEP_MICROSTEPS/2,motorParams.currentMa); //move one half step 'forward'
	delay_ms(100);
	A4950_move(A4950_STEP_MICROSTEPS,motorParams.currentMa); //move one half step 'forward'
	delay_ms(200);
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

	enableFeedback = feedback;
	A4950_enable(feedback);

	return ((float)x)/100.0;
}

stepCtrlError_t StepperCtrl_begin(void)
{
	float x=9999;
	enableFeedback = false;
	currentLocation = 0;

	//we have to update from NVM before moving motor
	StepperCtrl_updateParamsFromNVM(); //update the local cache from the NVM

	//start up encoder
	if (false == Encoder_begin())
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

		if (x < 0)
		{
			motorParams.motorWiring = !motorParams.motorWiring;
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

	setupTCInterrupts(SAMPLING_PERIOD_uS);

	return STEPCTRL_NO_ERROR;
}

void StepperCtrl_enable(bool enable) //enables feedback sensor processing StepperCtrl_processFeedback()
{
	if(StepperCtrl_Enabled == true && enable == false)
	{
		disableTCInterrupts();
	}
	if(StepperCtrl_Enabled == false && enable == true) //if we are enabling previous disabled motor
	{
		StepperCtrl_setLocationFromEncoder();
		enableTCInterrupts();
	}
	StepperCtrl_Enabled = enable;
}

void StepperCtrl_feedbackMode(uint8_t mode)
{
	disableTCInterrupts();
	switch (mode) //TODO add more modes
	{
	case STEPCTRL_OFF:
		enableFeedback = false;
		enableSoftOff = false;
		A4950_enable(false);
		break;
	case STEPCTRL_FEEDBACK_POSITION_RELATIVE:
		enableFeedback = true;
		enableCloseLoop = true;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_POSITION_ABSOLUTE: //TODO
		enableFeedback = true;
		enableCloseLoop = true;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_VELOCITY:	//TODO
		enableFeedback = true;
		enableCloseLoop = true;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_TORQUE:	//TODO
		enableFeedback = true;
		enableCloseLoop = false;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_CURRENT:	//TODO
		enableFeedback = true;
		enableCloseLoop = false;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF:
		enableFeedback = true;
		enableCloseLoop = false;
		enableSoftOff = true;
		break;
	default:
		enableFeedback = false;
		enableCloseLoop = false;
		A4950_enable(false);
		break;
	}
	enableTCInterruptsCond(StepperCtrl_Enabled);

	
}


bool StepperCtrl_processFeedback(void)
{
	bool no_error = false;
	int32_t desiredLoc;
	int32_t currentLoc;
	static int32_t lastLoc;
	const int16_t speed_filter_tc = 128; //speed filter time constant
	const int64_t error_filter_tc = 2; //error filter time constant - choose depending on CAN RX rate
	int32_t speed_raw;
	int32_t error;
	static int32_t desiredLoc_slow = 0;
	currentLoc = StepperCtrl_updateCurrentLocation(); //CurrentLocation
	desiredLoc = desiredLocation;

	loopError = desiredLoc - currentLoc;
	desiredLoc_slow = (desiredLoc + (error_filter_tc-1) * desiredLoc_slow) / error_filter_tc; 
	error = desiredLoc_slow - currentLoc; //error is desired - PoscurrentPos

	speed_raw = (currentLoc - lastLoc) * (int32_t) SAMPLING_HZ; // 360deg/65536/s
	speed_slow = (speed_raw + (speed_filter_tc-1) * speed_slow) / speed_filter_tc; 

	no_error = StepperCtrl_simpleFeedback(error);


	lastLoc = currentLoc;
	return no_error;
}

//this was written to do the PID loop not modeling a DC servo
// but rather using features of stepper motor.
bool StepperCtrl_simpleFeedback(int32_t error)
{
	static int32_t lastError = 0;
	static uint32_t errorCount = 0;
	const uint16_t smallLoad = 25; //abs mA

	static float iTerm; //iTerm memory
	static uint8_t saturationId = 2; //0 is negative saturation, 1 is positive saturation, 2 is no saturation
	static uint16_t magnitude = 0; //static for dTerm condition check

	if(enableFeedback)
	{
		int16_t loadAngleDesired;
		
		//todo add close loop intiazliation - I term with last control
		if(enableCloseLoop)
		{
			int32_t errorSat;
			int_fast16_t pTerm;
			int_fast16_t dTerm;
			
			//protect pTerm and iTerm 16bit variables against overflow
			int32_t errorMax = (int32_t) INT16_MAX * CTRL_PID_SCALING / max(sPID.Kp, sPID.Ki);  //todo: move this to StepperCtrl_updateParamsFromNVM()
			if( error > errorMax){
				errorSat = errorMax;
			}else if (error < -errorMax){
				errorSat = -errorMax;
			}else{
				errorSat = error;
			}

			// PID integral term
			if((errorSat>0) != saturationId){ //antiwindup clamp - condition optimized using clever id values (0,1,2) from previous sample //todo - actually... make it more explicit
				iTerm += (float) (errorSat * sPID.Ki / CTRL_PID_SCALING / (int32_t) SAMPLING_PERIOD_uS);
			}
			
			//error deadzone to reduce mechanical vibration due to P term
			if(abs(errorSat) < angleFullStep){
				errorSat = 0;  
			}

			// PID proportional term
			pTerm  =  errorSat * sPID.Kp / CTRL_PID_SCALING;

			// PID derivative term
			dTerm = (errorSat - lastError) * sPID.Kd / CTRL_PID_SCALING * (int32_t) SAMPLING_PERIOD_uS;
			if(magnitude < smallLoad){ //dTerm causes noise when motor is not loaded enough (previous magnitude)
				dTerm=0;
			}
			
			closeLoop = pTerm + (int16_t) iTerm + dTerm;

			// If closeLoop is in the same direction as feedforward, closelooop is limited by closeLoopMax. 
			// If closeLoop is in opposite direction to feedforward, then it is limited to -feedforward so that closeloop
			// has always power to cancel out the feedforward to avoid uncontrolled rotation

			if( closeLoop > max(closeLoopMax, -feedForward)) //todo: often spikes double the limit. Create smooth antiwindup
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
			control = closeLoop + feedForward;

			// add deadzone for small torque small error zero-crossing
			const int16_t zerocrossMax = 20; //mA
			if((control < zerocrossMax) && (control < -zerocrossMax) && (error < angleFullStep) && (error > -angleFullStep))
			{
				control = 0; //closeLoop * angleFullStep / zerocrossMax; //scale or just set to 0 if it keeps rocking
			}
			lastError = errorSat;

		}else if (enableSoftOff){
			static uint16_t rampsteps = 0;
			const uint8_t rampstepsMax = 1; //seconds
			if ((control != 0) && (rampsteps >= (SAMPLING_PERIOD_uS * rampstepsMax))){
				(control > 0) ? control-- : control++;
				rampsteps = 0;
			}else if (control == 0){
				A4950_enable(false);
			}
			rampsteps++;
			lastError = 0;
			closeLoop = 0;
		}
		else{
			control = feedForward;
			closeLoop = 0;
			lastError = 0;

			iTerm = 0;
			saturationId = 2;
		}
		

		if (control > 0)
		{
			loadAngleDesired = angleFullStep;
		}else if (control < 0)
		{
			loadAngleDesired = -angleFullStep;
		}else{
			loadAngleDesired = 0;
		}

		magnitude = (uint16_t) (fastAbs(control));

		int16_t loadAngleSpeedComp;//Compensate for angle sensor delay
	#ifdef MKS
		uint16_t sensDelay = 10u;
	#elif BTT
		uint16_t sensDelay = 90u;
	#else
		uint16_t sensDelay = 0u;
	#endif
		uint16_t angleSensLatency = (SAMPLING_PERIOD_uS + sensDelay);
		loadAngleSpeedComp = loadAngleDesired + (int16_t) (speed_slow * (int_fast16_t) angleSensLatency / (int32_t) S_to_uS  ); 
		StepperCtrl_moveToAngle(loadAngleSpeedComp, magnitude);
	
	}else{
		control = 0;
		closeLoop = 0;
		Iq_ma = 0;

		lastError = 0;
		iTerm = 0;
		saturationId = 2;
		magnitude = 0;
	}

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


void StepperCtrl_moveToAngle(int16_t loadAngle, uint16_t magnitude) 
{
	uint16_t absoluteAngle; //0-65535 -> 0-360
	uint16_t absoluteMicrosteps;

	absoluteAngle = (uint16_t) (((uint32_t) (currentLocation + loadAngle)) & ANGLE_MAX); //add load angle to current location
	absoluteMicrosteps = DIVIDE_WITH_ROUND((uint32_t) absoluteAngle *  motorParams.fullStepsPerRotation * A4950_STEP_MICROSTEPS, ANGLE_STEPS); //2^2=8 which is a common denominator of 200 and 256

	A4950_move(absoluteMicrosteps, magnitude);
}