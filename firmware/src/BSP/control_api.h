#ifndef CONTROL_API_H
#define CONTROL_API_H

#include <stdint.h>

void StepperCtrl_setDesiredLocation(int32_t deltaLocation);
void StepperCtrl_setFeedForwardTorque(int16_t Iq_feedforward);
void StepperCtrl_setCloseLoopTorque(uint16_t Iq_closeloopLim);
void StepperCtrl_setControlMode(uint8_t mode);


int32_t StepperCtrl_getCurrentLocation(void);
int16_t StepperCtrl_getCloseLoop(void);
int16_t StepperCtrl_getMotorIq(void);
int32_t StepperCtrl_getSpeedRev(void);
int32_t StepperCtrl_getPositionError(void);



#endif // CONTROL_API_H
