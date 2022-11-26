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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


/*
	@ Description:
	Simple interface to read StepperCtrl_processMotion() global variables
	All functions are supposed to be called from lesser priority task than StepperCtrl loop
	Todo: if needed, create snapshot of all variables before read so all are synced
*/
#include "control_api.h"
#include "board.h"
#include "stm32f10x.h"
#include "stepper_controller.h"
#include "actuator_config.h"
#include "nonvolatile.h"
#include "encoder.h"
#include "main.h"

#define DIR_SIGN(x) ((systemParams.dirRotation==CW_ROTATION) ? (x) : (-x))	//shorthand for swapping direction

void StepperCtrl_setDesiredAngle(float actuator_angle_delta){
	float newLocation = StepperCtrl_getAngleFromEncoderRaw() + DIR_SIGN(DEGREES_TO_ANGLERAW(actuator_angle_delta * gearing_ratio));
	//safe conversion
	int32_t newLocation_int;
	if (newLocation > INT32_MAX){
		newLocation_int = INT32_MAX;
	}else if (newLocation < INT32_MIN){
		newLocation_int = INT32_MIN;
	}else{
		newLocation_int = (int32_t)newLocation;
	}

	__disable_irq();
	desiredLocation = newLocation_int;
	__enable_irq();
}

void StepperCtrl_setFeedForwardTorque(float actuator_torque){ 
	float Iq_feedforward = DIR_SIGN(actuator_torque * actuatorTq_to_current); //convert actuator output torque to Iq current
	//safe conversion
	int_fast16_t Iq_feedforward_int;
	if(Iq_feedforward > INT_FAST16_MAX){
		Iq_feedforward_int = INT_FAST16_MAX;
	}else if(Iq_feedforward < INT_FAST16_MIN){
		Iq_feedforward_int = INT_FAST16_MIN;
	}else{
		Iq_feedforward_int = (int_fast16_t)Iq_feedforward;
	}
	__disable_irq();
	feedForward = Iq_feedforward_int;
	__enable_irq();
}

void StepperCtrl_setCloseLoopTorque(float actuator_torque_cl_max){ //set error correction max torque
	float Iq_closeloopLim = actuator_torque_cl_max * actuatorTq_to_current; //convert actuator output torque to Iq current
	//safe conversion
	int_fast16_t Iq_closeloopLim_int;
	if(Iq_closeloopLim > INT_FAST16_MAX){
		Iq_closeloopLim_int = INT_FAST16_MAX;
	}else if(Iq_closeloopLim < INT_FAST16_MIN){
		Iq_closeloopLim_int = INT_FAST16_MIN;
	}else{
		Iq_closeloopLim_int = (int_fast16_t)Iq_closeloopLim;
	}
	__disable_irq();
	closeLoopMax = Iq_closeloopLim_int;
	__enable_irq();
}

void StepperCtrl_setControlMode(uint8_t mode){ 
	__disable_irq();
	if ((stepCtrlError != STEPCTRL_NO_ERROR) || runCalibration){
		__enable_irq();
		return;
	}
	switch (mode){
		case 0:
			StepperCtrl_setMotionMode(STEPCTRL_OFF);
			break;
		case 1:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_TORQUE);
			break;
		case 2:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_POSITION_RELATIVE);
			break;
		default:
			StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
	__enable_irq();
}

//returns internal position integer
int32_t StepperCtrl_getAngleFromEncoderRaw(void) {
	int32_t ret;
	__disable_irq();
	ret = currentLocation;
	__enable_irq();
	return ret;
}

//returns actuator angle based in internal sensor
float StepperCtrl_getAngleFromEncoder(void) {
	return ANGLERAW_T0_DEGREES(DIR_SIGN(StepperCtrl_getAngleFromEncoderRaw())) / gearing_ratio;
}

//returns current close loop torque
float StepperCtrl_getCloseLoop(void) {
	int16_t ret;
	__disable_irq();
	ret = closeLoop;
	__enable_irq();
	return (float) DIR_SIGN(ret) * current_to_actuatorTq; //convert close loop control (mA) to actuator output torque
}

//returns current control actuator torque
float StepperCtrl_getControlOutput(void) {
	int16_t ret;
	__disable_irq();
	ret = control;
	__enable_irq();
	return DIR_SIGN(ret) * current_to_actuatorTq; //convert total control (mA) to actuator output torque
}

//returns current actuator speed in rev/s
float StepperCtrl_getSpeedRev(void) { //revolutions/s
	int32_t ret;
	__disable_irq();
	ret = speed_slow;
	__enable_irq();
	return (float) DIR_SIGN(ret) / ANGLE_STEPS / gearing_ratio; //convert speed angleraw/s to rev/s
}

//returns position error integer
float StepperCtrl_getPositionError(void) {
	int32_t ret;
	__disable_irq(); 
	ret = loopError;
	__enable_irq();
	return ANGLERAW_T0_DEGREES(DIR_SIGN(ret)) / gearing_ratio; //convert error (steps) to rev/s
}


extern volatile bool A4950_Enabled;
extern volatile uint32_t can_err_rx_cnt;

uint16_t StepperCtrl_getStatuses(void) {
	uint16_t ret1 = 0;
	uint16_t ret2 = 0;

	__disable_irq(); 
	// control loop status
	ret1 |= ((StepperCtrl_Enabled & 0x01) << 0);
	ret1 |= ((enableFeedback & 0x01) << 1);
	ret1 |= ((enableSoftOff & 0x01) << 2);
	ret1 |= ((enableCloseLoop & 0x01) << 4);
	__enable_irq();

	//debug - other
	ret2 |= ((A4950_Enabled & 0x01) << 0);
	ret2 |= ((motion_task_isr_enabled & 0x01) << 1); //here should be always 0

	// actuator parameters
	ret2 |= ((motorParams.motorWiring & 0x01) << 2);
	ret2 |= ((systemParams.dirRotation & 0x01) << 3);
	ret2 |= ((systemParams.errorPinMode & 0x01) << 4);

	//CAN checksum 
	ret2 |= (((can_err_rx_cnt > 0) & 0x01) << 5);
	
	//task status
	ret2 |= (((motion_task_overrun_count > 0) & 0x01) << 6);
	ret2 |= (((service_task_overrun_count > 0) & 0x01) << 7);
	
	return (ret2 << 8) | ret1 ;
}
