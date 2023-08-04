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
#include "utils.h"

static bool StepperCtrl_simpleFeedback(int32_t error);
static void StepperCtrl_moveToAngle(int16_t a, uint16_t ma);

volatile PID_t sPID; //simple control loop PID parameters
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

volatile bool StepperCtrl_Enabled = false;
volatile bool enableFeedback = false; //motor control using sensor angle feedback scheme
volatile bool enableCloseLoop = false; //true if control uses PID
volatile bool enableSoftOff = false; //true if soft off is enabled
static volatile bool enableRelative = true;
volatile int32_t angleFullStep = 327;

volatile int32_t zeroAngleOffset = 0;

//api - commanded
volatile int32_t desiredLocation;
volatile int16_t feedForward;
volatile int16_t closeLoopMaxDes;

//api - measured
volatile int32_t currentLocation = 0;
volatile int16_t closeLoop;
volatile int16_t control;
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


//Keep track of full rotations
//the current location lower 16 bits is angle (0-360 degrees in 65536 steps) while 
//upper 16 bits effectively hold number of full rotations.
static int32_t StepperCtrl_updateCurrentLocation(void)
{
	uint16_t angle = GetCorrectedAngle(ReadEncoderAngle());
	//convert to unsigned and use wrap around math to get circular angle distance!
	int16_t previousAngleDelta = (int16_t)(uint16_t)(angle - (uint16_t)((int16_t)(currentLocation % (int32_t)ANGLE_STEPS)));
	currentLocation = currentLocation + previousAngleDelta;

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
		//voltage check
		if (GetMotorVoltage() < 8.0f || GetSupplyVoltage() < 8.0f)
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
		enableRelative = true;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_POSITION_ABSOLUTE:
		enableFeedback = true;
		enableCloseLoop = true;
		enableRelative = false;

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


#define AVG_WINDOW 8U
static int32_t errMovingAverage(int32_t val) {
	const uint8_t window_n = AVG_WINDOW;
	static int32_t ringbuffer[AVG_WINDOW];
	static uint8_t ringbuffer_idx = 0;

	static int32_t avg=0;
	static int32_t accu=0;
	
	accu += val - ringbuffer[ringbuffer_idx];
	ringbuffer[ringbuffer_idx] = val;

	avg = accu/(int32_t)window_n;
	ringbuffer_idx = (ringbuffer_idx + 1U) % window_n;

	return avg;
}

bool StepperCtrl_processMotion(void)
{
	bool no_error = false;
	int32_t currentLoc;
	static int32_t lastLoc;
	const int16_t speed_filter_tc = 128; //speed filter time constant
	const int64_t error_filter_tc = 2; //error filter time constant - choose depending on CAN RX rate
	int32_t speed_raw;
	int32_t error;
	static int32_t desiredLoc_slow = 0;
	currentLoc = StepperCtrl_updateCurrentLocation(); //CurrentLocation

	loopError = desiredLocation - currentLoc;
	desiredLoc_slow = (desiredLocation + (error_filter_tc-1) * desiredLoc_slow) / error_filter_tc;

	if (enableRelative){
		error = desiredLoc_slow;
	}else{
		error = desiredLoc_slow - currentLoc; //error is desired - PoscurrentPos
	}

	speed_raw = (currentLoc - lastLoc) * (int32_t) SAMPLING_HZ; // deg/s*360/65536
	speed_slow = (speed_raw + (speed_filter_tc-1) * speed_slow) / speed_filter_tc; 

	int32_t error_flt = errMovingAverage(error);

	no_error = StepperCtrl_simpleFeedback(error_flt);

	lastLoc = currentLoc;
	return no_error;
}

//this was written to do the PID loop not modeling a DC servo
// but rather using features of stepper motor.
static bool StepperCtrl_simpleFeedback(int32_t error)
{
	static int32_t lastError = 0;
	static uint32_t errorCount = 0;

	static int32_t iTerm_accu; //iTerm memory - float to acumulate small and big errors
	static uint16_t magnitude = 0; //static for dTerm condition check

	if(enableFeedback) //todo add openloop control
	{
		int16_t loadAngleDesired = 0;
		
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
			int16_t pTerm;
			int16_t dTerm;

			int16_t closeLoopMax = closeLoopMaxDes;
			// increase closeLoopMax if feedForward is larger than it and in opposite direction
			// so that closeloop has always power to cancel out the feedforward to avoid uncontrolled rotation
			if((closeLoop > 0) && (-feedForward > closeLoopMaxDes)){
				closeLoopMax = -feedForward;
			}
			if((closeLoop < 0 ) && (feedForward > closeLoopMaxDes)){
				closeLoopMax = feedForward;
			}

			#define PID_TERMS 3
			int16_t maxEachTerm = closeLoopMax * PID_TERMS;

			int32_t errorMax = (int32_t)maxEachTerm * CTRL_PID_SCALING / sPID.Kp;
			//protect closeLoop against overflow and unrealistic values - due to P term
			if( error > errorMax){
				errorSat = errorMax;
			}else if (error < -errorMax){
				errorSat = -errorMax;
			}else{
				errorSat = error;
			}

			// PID - (I)ntegral term
			iTerm_accu += errorSat;
			int16_t iTerm = (int16_t)(iTerm_accu  * sPID.Ki / (int16_t)SAMPLING_PERIOD_uS / CTRL_PID_SCALING); //it's safe to cast to int16_t as iTerm_accu / CTRL_PID_SCALING cannot be much bigger than maxEachTerm since last time because iTerm_accu uses limited errorSat when acumulating error
			bool iTermLimited = false;
			//protect closeLoop against overflow and unrealistic values - due to I term
			if (iTerm  > maxEachTerm){
				iTerm = maxEachTerm;
				iTermLimited = true;
			}
			if (iTerm < -maxEachTerm){
				iTerm = -maxEachTerm;
				iTermLimited = true;
			}

			// PID - (P)roportional term
			// deadzone to reduce mechanical vibration of the P term
			if((errorSat < (angleFullStep/4)) && (errorSat > (-angleFullStep/4))){
				pTerm = 0;
			}else{
				pTerm  =  (int16_t)(errorSat * sPID.Kp / CTRL_PID_SCALING);
			}

			// PID - (D)erivative term
			// error deadzone to reduce mechanical vibration of the D term
			if(((error < angleFullStep) && (error > -angleFullStep))){
				dTerm=0;
			}else{
				int32_t deltaErrorMax = (int32_t) maxEachTerm * CTRL_PID_SCALING / sPID.Kd / (int16_t)SAMPLING_PERIOD_uS;
				int32_t deltaError = error - lastError;
				
				//protect closeLoop against overflow and unrealistic values - due to D term
				if( deltaError > deltaErrorMax){
					deltaError = deltaErrorMax;
				}
				if (deltaError < -deltaErrorMax){
					deltaError = -deltaErrorMax;
				}
				dTerm = (int16_t)(deltaError * sPID.Kd * (int16_t)SAMPLING_PERIOD_uS / CTRL_PID_SCALING);
			}
			lastError = error;
			
			
			closeLoop = pTerm + iTerm + dTerm;
			
			// Saturate against closeLoopMax - any excess subtract from integral part, but don't make it change sign
			if( closeLoop > closeLoopMax){	
				iTerm -= closeLoop - closeLoopMax;
				if(iTerm < 0){
					iTerm = 0;
				}
				iTermLimited = true;
				closeLoop = closeLoopMax;
			}
			if (closeLoop < -closeLoopMax){
				iTerm -= closeLoop - (-closeLoopMax);
				if(iTerm > 0){
					iTerm = 0;
				}
				iTermLimited = true;
				closeLoop = -closeLoopMax;
			}
			control = closeLoop + feedForward;

			// Saturate against I_MAX_A4950 - any excess subtract from integral part, but don't make it change sign
			if(control > I_MAX_A4950){	
				iTerm -= closeLoop - I_MAX_A4950;
				if(iTerm < 0){
					iTerm = 0;
				}
				iTermLimited = true;
				control = I_MAX_A4950;
			}
			if (control < -I_MAX_A4950){
				iTerm -= closeLoop - (-I_MAX_A4950);
				if(iTerm > 0){
					iTerm = 0;
				}
				iTermLimited = true;
				control = -I_MAX_A4950;
			}

			if(iTermLimited == true){ //backcalculate the accumulator
				iTerm_accu = (int32_t) SAMPLING_PERIOD_uS * CTRL_PID_SCALING * iTerm / sPID.Ki;
			}

		}else{
			control = feedForward;
			if(control > I_MAX_A4950){
				control = I_MAX_A4950;
			}
			if (control < -I_MAX_A4950){
				control = -I_MAX_A4950;
			}
			closeLoop = 0;
			lastError = 0;

			iTerm_accu = 0.0;
		}
		

		if (control > 0)
		{
			loadAngleDesired =  (int16_t) angleFullStep;
		}else if (control < 0)
		{
			loadAngleDesired = (int16_t) -angleFullStep;
		}else{
			loadAngleDesired = 0;
		}

		magnitude = (uint16_t) (fastAbs(control));

		int16_t loadAngleSpeedComp;//Compensate for angle sensor delay
		int16_t sensDelay = 90u;
		int16_t angleSensLatency = (int16_t)SAMPLING_PERIOD_uS + sensDelay;
		loadAngleSpeedComp = loadAngleDesired + (int16_t) (speed_slow * angleSensLatency / (int32_t) S_to_uS);
		StepperCtrl_moveToAngle(loadAngleSpeedComp, magnitude);
	
	}else{
		control = 0;
		closeLoop = 0;
		Iq_ma = 0;

		lastError = 0;
		iTerm_accu = 0.0;
		magnitude = 0;
	}

  // error needs to exist for some time period
	if ((lastError > angleFullStep) || (lastError < -angleFullStep))
	{
		++errorCount;
		if (errorCount > (SAMPLING_HZ >> 7))
		{
			return true;
		}
		return false;
	}

	if(errorCount > 0U)
	{
		--errorCount;
	}

	return false;
}


static void StepperCtrl_moveToAngle(int16_t loadAngle, uint16_t magnitude) 
{
	uint16_t absoluteAngle; //0-65535 -> 0-360
	uint16_t absoluteMicrosteps;

	absoluteAngle = (uint16_t) (((uint32_t) (currentLocation + loadAngle)) & ANGLE_MAX); //add load angle to current location
	absoluteMicrosteps = absoluteAngle *  motorParams.fullStepsPerRotation * A4950_STEP_MICROSTEPS / ANGLE_STEPS; //2^2=8 which is a common denominator of 200 and 256

	A4950_move(absoluteMicrosteps, magnitude);
}