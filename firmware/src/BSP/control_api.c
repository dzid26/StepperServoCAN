/*
	Simple interface to read StepperCtrl_processFeedback() global variables
	All functions are supposed to be called from lesser priority task than StepperCtrl loop
	Todo: if needed, create snapshot of all variables before read so all are synced
*/
#include "control_api.h"

#include "stepper_controller.h"
#include "actuator_config.h"

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

	disableTCInterrupts();
	desiredLocation = newLocation_int;
	enableTCInterruptsCond(StepperCtrl_Enabled);
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
	disableTCInterrupts();
	feedForward = Iq_feedforward_int;
	enableTCInterruptsCond(StepperCtrl_Enabled);
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
	disableTCInterrupts();
	closeLoopMax = Iq_closeloopLim_int;
	enableTCInterruptsCond(StepperCtrl_Enabled);
}

void StepperCtrl_setControlMode(uint8_t mode){ 
	if (stepCtrlError != STEPCTRL_NO_ERROR){
		return;
	}
	switch (mode){
		case 0:
			StepperCtrl_feedbackMode(STEPCTRL_OFF);
			break;
		case 1:
			StepperCtrl_feedbackMode(STEPCTRL_FEEDBACK_TORQUE);
			break;
		case 2:
			StepperCtrl_feedbackMode(STEPCTRL_FEEDBACK_POSITION_RELATIVE);
			break;
		default:
			StepperCtrl_feedbackMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
}

//returns internal position integer
int32_t StepperCtrl_getAngleFromEncoderRaw(void) {
	int32_t ret;
	disableTCInterrupts();
	ret = currentLocation;
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ret;
}

//returns actuator angle based in internal sensor
float StepperCtrl_getAngleFromEncoder(void) {
	return ANGLERAW_T0_DEGREES(DIR_SIGN(StepperCtrl_getAngleFromEncoderRaw())) / gearing_ratio;
}

//returns current close loop torque
float StepperCtrl_getCloseLoop(void) {
	int16_t ret;
	disableTCInterrupts(); 
	ret = closeLoop;
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return (float) DIR_SIGN(ret) * current_to_actuatorTq; //convert close loop control (mA) to actuator output torque
}

//returns current control actuator torque
float StepperCtrl_getControlOutput(void) {
	int16_t ret;
	disableTCInterrupts(); 
	ret = control;
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return DIR_SIGN(ret) * current_to_actuatorTq; //convert total control (mA) to actuator output torque
}

//returns current actuator speed in rev/s
float StepperCtrl_getSpeedRev(void) { //revolutions/s
	int32_t ret;
	disableTCInterrupts(); 
	ret = speed_slow;
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return (float) DIR_SIGN(ret) / ANGLE_STEPS / gearing_ratio; //convert speed angleraw/s to rev/s
}

//returns position error integer
float StepperCtrl_getPositionError(void) {
	int32_t ret;
	disableTCInterrupts(); 
	ret = loopError;
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ANGLERAW_T0_DEGREES(DIR_SIGN(ret)) / gearing_ratio; //convert error (steps) to rev/s
}


extern volatile bool A4950_Enabled;
extern volatile uint32_t can_err_rx_cnt;

uint16_t StepperCtrl_getStatuses(void) {
	uint16_t ret1 = 0;
	uint16_t ret2 = 0;

	disableTCInterrupts(); 
	// control loop status
	ret1 |= ((StepperCtrl_Enabled & 0x01) << 0);
	ret1 |= ((enableFeedback & 0x01) << 1);
	ret1 |= ((enableSoftOff & 0x01) << 2);
	ret1 |= ((enableCloseLoop & 0x01) << 4);
	enableTCInterruptsCond(StepperCtrl_Enabled);

	//debug - other
	ret2 |= ((A4950_Enabled & 0x01) << 0);
	ret2 |= ((TC1_ISR_Enabled & 0x01) << 1); //here should be always 0

	// actuator parameters
	ret2 |= ((motorParams.motorWiring & 0x01) << 2);
	ret2 |= ((systemParams.dirRotation & 0x01) << 3);
	ret2 |= ((systemParams.errorPinMode & 0x01) << 4);

	//CAN checksum 
	ret2 |= (((can_err_rx_cnt > 0) & 0x01) << 5);
	
	//task status
	ret2 |= (((Task_Motor_overrun_count > 0) & 0x01) << 6);
	ret2 |= (((Task_10ms_overrun_count > 0) & 0x01) << 7);
	
	return (ret2 << 8) | ret1 ;
}
