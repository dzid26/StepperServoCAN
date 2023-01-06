 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
 *
 * Copyright (C) 2018 MisfitTech LLC.
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
 * along with this program.  If not, see <www.gnu.org/licenses/>.
 *
 */

#include "stepper_controller.h"
#include "nonvolatile.h"
#include "A4950.h"
#include "board.h"
#include "encoder.h"
#include "math.h"

static void StepperCtrl_moveToAngle(int16_t loadAngle, int16_t current_target);

volatile PID_t sPID; //simple control loop PID parameters
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

volatile bool StepperCtrl_Enabled = false;
volatile bool enableFeedback = false; //motor control using sensor angle feedback scheme
volatile bool enableCloseLoop = false; //true if control uses PID
volatile bool enableSoftOff = false; //true if soft off is enabled
volatile int32_t angleFullStep = 327;

volatile int32_t zeroAngleOffset = 0;

//api - commanded
volatile int32_t desiredLocation;
volatile int_fast16_t feedForward;
volatile int_fast16_t closeLoopMax=1000;

//api - measured
volatile int32_t currentLocation = 0;
volatile int_fast16_t closeLoop;
volatile int_fast16_t control;
volatile int16_t Iq_ma;
volatile int32_t speed_slow = 0;
volatile int32_t loopError = 0;

static void StepperCtrl_updateParamsFromNVM(void)
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

		motorParams.currentHoldMa = 400;
		motorParams.currentMa = 800;
	}
}


void StepperCtrl_setLocationFromEncoder(void)
{
	currentLocation = 0;

	if (CalibrationTable_calValid()){
		currentLocation = (int32_t)GetCorrectedAngle(OverSampleEncoderAngle(100U)); //save position
	}
	desiredLocation = StepperCtrl_updateCurrentLocation(); //zero the angle shown on LCD
}

//estimate of the current location from encoder feedback
//the current location lower 16 bits is angle (0-360 degrees in 65536 steps) while upper
//bits is the number of full rotations.
int32_t StepperCtrl_updateCurrentLocation(void)
{
	uint16_t angle = GetCorrectedAngle(ReadEncoderAngle());
	//use unisgned wrap around math to get circular angle distance
	uint16_t angleDelta = angle - (uint16_t)((int16_t)(currentLocation%(int32_t)ANGLE_STEPS));
	currentLocation = currentLocation + (int16_t)angleDelta;

	return currentLocation;
}


stepCtrlError_t StepperCtrl_begin(void)
{
	float x=9999.0f;
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


	if (NVM->motorParams.parametersValid == valid)
	{
		//lets read the motor voltage
		if (GetMotorVoltage() < 8.0f)
		{
			return STEPCTRL_NO_POWER;
		}
	}else
	{
		x = StepperCtrl_measureStepSize();
		if (fabs(x) < 0.5f)
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

		if (x < 0.0f)
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
		A4950_move(0, 0); //release the motor
		nvmParams.motorParams = motorParams;
		nvmWriteConfParms(&nvmParams);
	}

	angleFullStep = (int32_t)(ANGLE_STEPS / motorParams.fullStepsPerRotation);

	StepperCtrl_setLocationFromEncoder(); //measure new starting point

	if (false == CalibrationTable_calValid())
	{
		return STEPCTRL_NO_CAL;
	}

	Motion_task_init(SAMPLING_PERIOD_uS);

	return STEPCTRL_NO_ERROR;
}

void StepperCtrl_enable(bool enable) //enables feedback sensor processing StepperCtrl_processMotion()
{
	if(StepperCtrl_Enabled == true && enable == false)
	{
		Motion_task_disable();
		//reset globals:
		speed_slow = 0;
		closeLoop = 0;
		control = 0;
		Iq_ma = 0;
		currentLocation = 0;
	}
	if(StepperCtrl_Enabled == false && enable == true) //if we are enabling previous disabled motor
	{
		StepperCtrl_setLocationFromEncoder();
		Motion_task_enable();
	}
	StepperCtrl_Enabled = enable;
}

void StepperCtrl_setMotionMode(uint8_t mode)
{
	switch (mode) //TODO add more modes
	{
	case STEPCTRL_OFF:
		enableFeedback = false; //motor control using angle sensor feedback is off
		// enableNoFeedback = false; //motor control fallback to open loop
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
}

int32_t movingAverage(int32_t value) {
	const uint8_t statesize = 8;
	static int32_t ringbuffer[8]; //ringbuffer[statesize] in C++
	static int32_t integrator = 0;
	// don't forget to initialize the ringbuffer somehow
	static uint8_t ringbuffer_ptr = 0;
	
	int32_t oldest_value = ringbuffer[ringbuffer_ptr];
	integrator += (value - oldest_value);
	ringbuffer[ringbuffer_ptr] = value;
	ringbuffer_ptr = (ringbuffer_ptr+1u) % statesize;

	return integrator / (int8_t)statesize;
}


bool StepperCtrl_processMotion(void)
{
	bool no_error = false;
	int32_t commandedLoc;
	int32_t currentLoc;
	static int32_t lastLoc;
	const int16_t speed_filter_tc = 128; //speed filter time constant
	const int64_t error_filter_tc = 2; //error filter time constant - choose depending on CAN RX rate
	int32_t speed_raw;
	int32_t error;
	static int32_t desiredLoc_slow = 0;
	currentLoc = StepperCtrl_updateCurrentLocation(); //CurrentLocation
	commandedLoc = desiredLocation;

	loopError = commandedLoc - currentLoc;
	desiredLoc_slow = (commandedLoc + (error_filter_tc-1) * desiredLoc_slow) / error_filter_tc; 
	error = desiredLoc_slow - currentLoc; //error is desired - PoscurrentPos

	speed_raw = (currentLoc - lastLoc) * (int32_t) SAMPLING_HZ; // deg/s*360/65536
	speed_raw = movingAverage(speed_raw);
	speed_slow = (speed_raw + (speed_filter_tc-1) * speed_slow) / speed_filter_tc; 

	no_error = StepperCtrl_simpleFeedback(error);


	lastLoc = currentLoc;
	return no_error;
}

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    (_a > _b) ? _a : _b;       \
})

//this was written to do the PID loop not modeling a DC servo
// but rather using features of stepper motor.
bool StepperCtrl_simpleFeedback(int32_t error)
{
	static int32_t lastError = 0;
	static uint32_t errorCount = 0;
	const uint16_t smallLoad = 25; //abs mA

	static float iTerm; //iTerm memory
	static uint8_t saturationId = 2; //0 is negative saturation, 1 is positive saturation, 2 is no saturation

	if(enableFeedback) //todo add openloop control
	{
		int16_t loadAngleDesired;
		
		//todo add close loop intiazliation - I term with last control
		if (enableSoftOff){
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
		else if(enableCloseLoop){
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
			if(control < smallLoad && control > -smallLoad){ //dTerm causes noise when motor is not loaded enough (previous magnitude)
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

		}else{
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


		int16_t loadAngleSpeedComp;//Compensate for angle sensor delay
		uint16_t sensDelay = 45u;
		uint16_t angleSensLatency = sensDelay;
		loadAngleSpeedComp = loadAngleDesired + (int16_t) (speed_slow * (int_fast16_t) angleSensLatency / (int32_t) S_to_uS  ); 
		StepperCtrl_moveToAngle(loadAngleSpeedComp, control);
	
	}else{
		control = 0;
		closeLoop = 0;
		Iq_ma = 0;

		lastError = 0;
		iTerm = 0;
		saturationId = 2;
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


static void StepperCtrl_moveToAngle(int16_t loadAngle, int16_t current_target) 
{
	uint16_t absoluteAngle; //0-65535 -> 0-360
	uint16_t absoluteMicrosteps;

	absoluteAngle = (uint16_t) (((uint32_t) (currentLocation + loadAngle)) & ANGLE_MAX); //add load angle to current location
	absoluteMicrosteps = absoluteAngle *  motorParams.fullStepsPerRotation * A4950_STEP_MICROSTEPS / ANGLE_STEPS; //2^2=8 which is a common denominator of 200 and 256

	if(1)
		A4950_move_volt(absoluteMicrosteps, current_target);
	else
		A4950_move(absoluteMicrosteps, (uint16_t)fastAbs(current_target));
}