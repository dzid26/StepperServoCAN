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
 
#ifndef __STEPPER_CONTROLLER_H
#define __STEPPER_CONTROLLER_H

#include "A4950.h"
#include "delay.h"
#include "nonvolatile.h"
#include "calibration.h"
#include "A1333.h"
#include "math.h"

typedef enum {
	STEPCTRL_NO_ERROR=0,
	STEPCTRL_NO_POWER=1, 	//no power to motor
	STEPCTRL_NO_CAL=2,	 	//calibration not set
	STEPCTRL_NO_ENCODER=3,//encoder not working
} stepCtrlError_t;

typedef struct {
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
} PID_t;

#define ANGLE_FROM_DEGREES(x) ((int32_t) ( ((float)ANGLE_STEPS * (float)(x)+ 180.0) / 360.0 ) )
#define ANGLE_T0_DEGREES(x) ( (float) ((float(x) * 360.0) / ((float)ANGLE_STEPS) ))

//scales PID parameters from Flash (floating point) to int32_t used in control 
#define CTRL_PID_SCALING 			(uint16_t)(1024)

#define MHz_to_Hz	(uint32_t)(1000000)
#define S_to_uS   	MHz_to_Hz
#define SAMPLING_PERIOD_uS	(uint16_t)(62) //sampling time in uS of control loop
#define SAMPLING_HZ		(uint16_t)(1u * S_to_uS / SAMPLING_PERIOD_uS) //update rate of control loop

void setupTCInterrupts(void);
void enableTCInterrupts(void);
void disableTCInterrupts(void);
void  StepperCtrl_motorReset(void);
void StepperCtrl_setLocationFromEncoder(void);
int32_t StepperCtrl_updateCurrentLocation(void);
void StepperCtrl_updateDesiredLocation(int32_t change);
int32_t StepperCtrl_getDesiredLocation(void);
uint16_t StepperCtrl_calibrateEncoder(bool update);
static uint16_t CalibrationMove(bool updateFlash, int8_t dir, int32_t *microSteps, uint8_t *passes, uint16_t calLocOffset);
uint16_t StepperCtrl_sampleMeanEncoder(uint16_t numSamples);
uint16_t StepperCtrl_getEncoderAngle(void);
void StepperCtrl_updateParamsFromNVM(void);
void StepperCtrl_setRotationDirection(bool forward);
float StepperCtrl_measureStepSize(void);
stepCtrlError_t StepperCtrl_begin(void);
void StepperCtrl_enable(bool enable);
bool StepperCtrl_processFeedback(void);
bool StepperCtrl_simpleFeedback(int32_t error);
void StepperCtrl_moveToAngle(int32_t a, uint16_t ma);

void set_StepperSteps(int32_t steps);

#endif
