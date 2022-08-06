#ifndef CONTROL_API_H
#define CONTROL_API_H

#include <stdint.h>

void StepperCtrl_setDesiredAngle(float deltaLocation);
void StepperCtrl_setFeedForwardTorque(float Iq_feedforward);
void StepperCtrl_setCloseLoopTorque(float Iq_closeloopLim);
void StepperCtrl_setControlMode(uint8_t mode);


int32_t StepperCtrl_getAngleFromEncoderRaw(void);
float StepperCtrl_getAngleFromEncoder(void);
float StepperCtrl_getCloseLoop(void);
float StepperCtrl_getControlOutput(void);
float StepperCtrl_getSpeedRev(void);
float StepperCtrl_getPositionError(void);
uint16_t StepperCtrl_getStatuses(void);


#endif // CONTROL_API_H
