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
#include "actuator_config.h"
#include "A4950.h"
#include "board.h"
#include "encoder.h"
#include "motor.h"
#include "utils.h"

volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

volatile bool StepperCtrl_Enabled = false;
volatile bool enableSensored = false; //motor control using sensor angle feedback scheme
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
volatile int16_t control_actual;
volatile int32_t speed_slow = 0; // rev/s/65536
volatile int32_t loopError = 0;

// special mode
static bool base_speed_mode = false;

static void UpdateRuntimeParams(void)
{
	//copy nvm (flash) to ram for fast access
	pPID.Kp = nvmMirror.pPID.Kp * CTRL_PID_SCALING;
	pPID.Ki = nvmMirror.pPID.Ki * CTRL_PID_SCALING;
	pPID.Kd = nvmMirror.pPID.Kd * CTRL_PID_SCALING;

	vPID.Kp = nvmMirror.vPID.Kp * CTRL_PID_SCALING;
	vPID.Ki = nvmMirror.vPID.Ki * CTRL_PID_SCALING;
	vPID.Kd = nvmMirror.vPID.Kd * CTRL_PID_SCALING;

	liveSystemParams = nvmMirror.systemParams;
	liveMotorParams = nvmMirror.motorParams;
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


stepCtrlError_t StepperCtrl_begin(void){
	enableSensored = false;
	currentLocation = 0;

	//update the runtime storage from the NVM
	UpdateRuntimeParams();

	//start up encoder
	if (false == Encoder_begin())
	{
		return STEPCTRL_NO_ENCODER;
	}

	//cal table init
	CalibrationTable_init();

	//voltage check
	if ((GetSupplyVoltage() < MIN_SUPPLY_VOLTAGE) || (GetMotorVoltage() < MIN_SUPPLY_VOLTAGE - 0.1f)){
		return STEPCTRL_NO_POWER;
	}

	angleFullStep = (int32_t)(ANGLE_STEPS / liveMotorParams.fullStepsPerRotation);

	if ((liveMotorParams.fullStepsPerRotation == FULLSTEPS_NA) || (CalibrationTable_calValid() == false)) {
		return STEPCTRL_NO_CAL;
	}
	assert((liveMotorParams.fullStepsPerRotation == FULLSTEPS_1_8) || (liveMotorParams.fullStepsPerRotation == FULLSTEPS_0_9));

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
		control_actual = 0;
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
	//refresh parameters when exiting STEPCTRL_OFF state
	static uint8_t mode_prev = STEPCTRL_OFF;
	if((mode != STEPCTRL_OFF) && (mode_prev != mode)){
		UpdateRuntimeParams();
	}
	base_speed_mode = false;
	switch (mode) {
	case STEPCTRL_OFF:
		enableSensored = false; //motor control using angle sensor feedback is off
		// enableNoFeedback = false; //motor control fallback to open loop
		enableSoftOff = false;
		A4950_enable(false);
		break;
	case STEPCTRL_FEEDBACK_POSITION_RELATIVE: //TODO
		enableSensored = true;
		enableCloseLoop = true;
		enableRelative = true;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_POSITION_ABSOLUTE:
		enableSensored = true;
		enableCloseLoop = true;
		enableRelative = false;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_VELOCITY:	//TODO
		enableSensored = true;
		enableCloseLoop = true;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_TORQUE:
		enableSensored = true;
		enableCloseLoop = false;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_CURRENT:	//TODO
		enableSensored = true;
		enableCloseLoop = false;
		A4950_enable(true);
		break;
	case STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF:
		enableSensored = true;
		enableCloseLoop = false;
		enableSoftOff = true;
		break;
	case STEPCTRL_FEEDBACK_KBEMF_ADAPT:
		base_speed_mode = true;
		A4950_enable(true);
		break;
	default:
		enableSensored = false;
		enableCloseLoop = false;
		A4950_enable(false);
		break;
	}
	mode_prev = mode;
}

void StepperCtrl_setCurrent(int16_t current){
	feedForward = current;
}

void StepperCtrl_setCloseLoopCurrentLim(int16_t current){
	closeLoopMaxDes = current;
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
	const int16_t speed_filter_tc = 8; //speed filter time constant
	const int8_t error_filter_tc = 2; //error filter time constant - choose depending on CAN RX rate
	int32_t speed_raw;
	int32_t error;
	static int32_t desiredLoc_slow = 0;
	currentLoc = StepperCtrl_updateCurrentLocation(); //CurrentLocation

	loopError = desiredLocation - currentLoc;
	speed_raw = (currentLoc - lastLoc) * (int32_t) SAMPLING_HZ; // rev/s/65536
	speed_slow = (int32_t)(int64_t)((speed_raw + (int64_t)(int16_t)(speed_filter_tc-1) * speed_slow) / speed_filter_tc);
	lastLoc = currentLoc;

	desiredLoc_slow = (int32_t)(int64_t)(desiredLocation + (int64_t)(int8_t)(error_filter_tc-1) * desiredLoc_slow) / error_filter_tc;
	if (enableRelative){
		error = desiredLoc_slow;
	}else{
		error = desiredLoc_slow - currentLoc; //error is desired - PoscurrentPos
	}
	int32_t error_flt = errMovingAverage(error);
	static int32_t lastError = 0;
	static uint32_t errorCount = 0;

	static int32_t iTerm_accu; //iTerm memory

	if (base_speed_mode){
		control = feedForward;
		base_speed_test(control);
	}else if(enableSensored){ //todo add openloop control
		if (enableSoftOff){
			if (control != 0) {
				// Calculate the number of tick required to ramp down at DESIRED_TORQUE_RATE
				#define DESIRED_TORQUE_RATE 2U // Nm/s
				const uint16_t tick_max = ((uint16_t)SAMPLING_HZ / DESIRED_TORQUE_RATE / (uint16_t)actuatorTq_to_current);
				// count tick between reduction
				static uint16_t tick = 0;
				tick++;
				if (tick >= tick_max) {
					if (control > 0) {
						--control;
					} else {
						++control;
					}
					tick = 0;
				}
			} else {
				A4950_enable(false);
			}
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

			int32_t errorMax = (int32_t)maxEachTerm * CTRL_PID_SCALING / pPID.Kp;
			//protect closeLoop against overflow and unrealistic values - due to P term
			errorSat = clip(error, -errorMax, errorMax);

			// PID - (I)ntegral term
			iTerm_accu += errorSat;
			int16_t iTerm = (int16_t)(iTerm_accu  * pPID.Ki / (int16_t)SAMPLING_PERIOD_uS / CTRL_PID_SCALING); //it's safe to cast to int16_t as iTerm_accu / CTRL_PID_SCALING cannot be much bigger than maxEachTerm since last time because iTerm_accu uses limited errorSat when acumulating error

			//protect closeLoop against overflow and unrealistic values - due to I term
			iTerm = clip(iTerm, -maxEachTerm, maxEachTerm);
			// PID - (P)roportional term
			// deadzone to reduce mechanical vibration of the P term
			if((errorSat < (angleFullStep/4)) && (errorSat > (-angleFullStep/4))){
				pTerm = 0;
			}else{
				pTerm  =  (int16_t)(errorSat * pPID.Kp / CTRL_PID_SCALING);
			}

			// PID - (D)erivative term
			// error deadzone to reduce mechanical vibration of the D term
			if(((error < angleFullStep) && (error > -angleFullStep))){
				dTerm=0;
			}else{
				int32_t deltaErrorMax = (int32_t) maxEachTerm * CTRL_PID_SCALING / pPID.Kd / (int16_t)SAMPLING_PERIOD_uS;
				int32_t deltaError = error - lastError;
				
				//protect closeLoop against overflow and unrealistic values - due to D term
				deltaError = clip(deltaError, -deltaErrorMax, deltaErrorMax);
				dTerm = (int16_t)(deltaError * pPID.Kd * (int16_t)SAMPLING_PERIOD_uS / CTRL_PID_SCALING);
			}
			lastError = error;

			closeLoop = pTerm + iTerm + dTerm;
			
			// Saturate against closeLoopMax - any excess subtract from integral part, but don't make it change sign
			int16_t closeLoopSat = clip(closeLoop, -closeLoopMax, closeLoopMax);
			iTerm -= clip(closeLoop - closeLoopSat, -iTerm, iTerm);

			// add feedforward
			int16_t controlSum = closeLoopSat + feedForward;
 
			// Saturate against MAX_CURRENT - any excess subtract from integral part, but don't make it change sign
			control = clip(controlSum, -MAX_CURRENT, MAX_CURRENT);
			iTerm -= clip(controlSum - control, -iTerm, iTerm);

			iTerm_accu = (int32_t) SAMPLING_PERIOD_uS * CTRL_PID_SCALING * iTerm / pPID.Ki;

		}else{
			control = clip(feedForward, -MAX_CURRENT, MAX_CURRENT);
			closeLoop = 0;
			lastError = 0;

			iTerm_accu = 0;
		}

		field_oriented_control(control);

	}else{
		control = 0;
		closeLoop = 0;
		control_actual = 0;

		lastError = 0;
		iTerm_accu = 0;
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
