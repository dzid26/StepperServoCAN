/**
 * StepperServoCAN
 *
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


/*
	@ Description:
	Simple interface to read StepperCtrl_processMotion() global variables
	All functions are supposed to be called from lesser priority task than StepperCtrl loop
*/
#include "control_api.h"
#include "board.h"
#include "stm32f10x.h"
#include "stepper_controller.h"
#include "actuator_config.h"
#include "nonvolatile.h"
#include "encoder.h"
#include "main.h"
#include "Msg.h"

#define DIR_SIGN(x) ((liveSystemParams.dirRotation==CW_ROTATION) ? (x) : -(x))	//shorthand for swapping direction

static bool api_allow_control = false;

void apiAllowControl(bool allow) {
	api_allow_control = allow;
}

void StepperCtrl_setDesiredAngle(float actuator_angle_delta){
	float newLocation = roundf(DIR_SIGN(DEGREES_TO_ANGLERAW(actuator_angle_delta * gearing_ratio)));
	//safe conversion to INT
	int32_t newLocation_int;
	if (newLocation > INT32_MAX){ // INT32_MAX will be represented as INT32_MAX+1, but it still works when casting to int32_t
		newLocation_int = INT32_MAX;
	}else if (newLocation < INT32_MIN){
		newLocation_int = INT32_MIN;
	}else{
		newLocation_int = (int32_t)newLocation;
	}
	
	if (api_allow_control) {
		desiredLocation = newLocation_int;
	}
}

//sets torque [Nm] in motion control loop 
void StepperCtrl_setFeedForwardTorque(float actuator_torque){ 
	float Iq_feedforward = roundf(DIR_SIGN(actuator_torque * actuatorTq_to_current)); //convert actuator output torque to Iq current
	//safe conversion
	int16_t Iq_feedforward_int;
	if(Iq_feedforward > INT16_MAX){
		Iq_feedforward_int = INT16_MAX;
	}else if(Iq_feedforward < INT16_MIN){
		Iq_feedforward_int = INT16_MIN;
	}else{
		Iq_feedforward_int = (int16_t)Iq_feedforward;
	}

	if (api_allow_control) {
		StepperCtrl_setCurrent(Iq_feedforward_int);
	}
}

//sets max close loop torque [Nm] in motion control loop 
void StepperCtrl_setCloseLoopTorque(float actuator_torque_cl_max){ //set error correction max torque
	float Iq_closeloopLim = roundf(actuator_torque_cl_max * actuatorTq_to_current); //convert actuator output torque to Iq current
	//safe conversion
	int16_t Iq_closeloopLim_int;
	if(Iq_closeloopLim > INT16_MAX){
		Iq_closeloopLim_int = INT16_MAX;
	}else if(Iq_closeloopLim < INT16_MIN){
		Iq_closeloopLim_int = INT16_MIN;
	}else{
		Iq_closeloopLim_int = (int16_t)Iq_closeloopLim;
	}

	if (api_allow_control) {
		StepperCtrl_setCloseLoopCurrentLim(Iq_closeloopLim_int);
	}
}

static bool safetyLockoutEnabled = false;
void StepperCtrl_setControlMode(uint8_t mode){ 
	if (stepCtrlError != STEPCTRL_NO_ERROR) {
		return;
	}

	if (!api_allow_control){
		return;
	}

	// block commands if SoftOff was enabled and torque reached 0
	if (enableSoftOff && control_actual == 0 && mode != MSG_STEERING_COMMAND_STEER_MODE_OFF_CHOICE) {
		safetyLockoutEnabled = true;
		return;
	}
	if (safetyLockoutEnabled && mode == MSG_STEERING_COMMAND_STEER_MODE_OFF_CHOICE) {
		safetyLockoutEnabled = false;
	}

	switch (mode){
		case MSG_STEERING_COMMAND_STEER_MODE_OFF_CHOICE:
			StepperCtrl_setMotionMode(STEPCTRL_OFF);
			break;
		case MSG_STEERING_COMMAND_STEER_MODE_TORQUE_CONTROL_CHOICE:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_TORQUE);
			break;
		case MSG_STEERING_COMMAND_STEER_MODE_ANGLE_CONTROL_CHOICE:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_POSITION_ABSOLUTE);
			break;
		case MSG_STEERING_COMMAND_STEER_MODE_SOFT_OFF_CHOICE:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
			break;
		default:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
}

//returns internal position integer
int32_t StepperCtrl_getAngleFromEncoderRaw(void) {
	int32_t ret;
	ret = currentLocation;
	return ret;
}

//returns actuator angle based in internal sensor
float StepperCtrl_getAngleFromEncoder(void) {
	return ANGLERAW_T0_DEGREES(DIR_SIGN(StepperCtrl_getAngleFromEncoderRaw())) / gearing_ratio;
}

//returns current close loop torque
float StepperCtrl_getCloseLoop(void) {
	int16_t ret;
	ret = closeLoop;
	return (float) DIR_SIGN(ret) * current_to_actuatorTq; //convert close loop control (mA) to actuator output torque
}

//returns current control actuator torque
float StepperCtrl_getControlOutput(void) {
	int16_t ret;
	ret = control_actual;
	return DIR_SIGN(ret) * current_to_actuatorTq; //convert total control (mA) to actuator output torque
}

//returns current actuator speed in rev/s
float StepperCtrl_getSpeedRev(void) { //revolutions/s
	int32_t ret;
	ret = speed_slow;
	return (float) DIR_SIGN(ret) / ANGLE_STEPS / gearing_ratio; //convert speed angleraw/s to rev/s
}

//returns position error integer
float StepperCtrl_getPositionError(void) {
	int32_t ret; 
	ret = loopError;
	return ANGLERAW_T0_DEGREES(DIR_SIGN(ret)) / gearing_ratio; //convert error (steps) to rev/s
}


extern volatile bool driverEnabled;
extern volatile uint32_t can_err_rx_cnt;

uint16_t StepperCtrl_getStatuses(void){
	uint8_t ret1 = 0;
	uint8_t ret2 = 0;
 
	// control loop status
	ret1 |= (controlsActive ? 0x1U : 0x0U) << 0U;
	ret1 |= (enableSensored ? 0x1U : 0x0U) << 1U;
	ret1 |= (safetyLockoutEnabled ? 0x1U : 0x0U) << 2U;
	ret1 |= (enableCloseLoop ? 0x1U : 0x0U) << 3U;

	//debug - other
	ret2 |= (driverEnabled ? 0x1U : 0x0U) << 0U;
	ret2 |= (motion_task_isr_enabled ? 0x1U : 0x0U) << 1U; //here should be always 0

	// actuator parameters
	ret2 |= (liveMotorParams.invertedPhase ? 0x1U : 0x0U) << 2U;
	ret2 |= (liveSystemParams.dirRotation ? 0x1U : 0x0U) << 3U;
	ret2 |= (liveSystemParams.errorPinMode ? 0x1U : 0x0U) << 4U;

	//CAN checksum 
	ret2 |= ((can_err_rx_cnt > 0U) ? 0x1U : 0x0U) << 5U;
	
	//task status
	ret2 |= ((motion_task_overrun_count > 0U) ? 0x1U : 0x0U) << 6U;
	ret2 |= ((service_task_overrun_count > 0U) ? 0x1U : 0x0U) << 7U;
	
	return (uint16_t)((uint16_t)ret2 << 8U) | (uint16_t)ret1;
}
