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
