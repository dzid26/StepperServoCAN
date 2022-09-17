 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#ifndef __STEPPER_CONTROLLER_H
#define __STEPPER_CONTROLLER_H

#include "delay.h"
#include "nonvolatile.h"
#include "calibration.h"
#include "tle5012.h"
#include "math.h"

typedef enum {
	STEPCTRL_NO_ERROR=0,
	STEPCTRL_NO_POWER=1, 	//no power to motor
	STEPCTRL_NO_CAL=2,	 	//calibration not set
	STEPCTRL_NO_ENCODER=3,//encoder not working
} stepCtrlError_t;

typedef	enum {
	STEPCTRL_OFF=0,
	//Uses angle sensor to control the stepper load
	STEPCTRL_FEEDBACK_POSITION_RELATIVE=1,	//relative closeloop positioning with feedforward torque
	STEPCTRL_FEEDBACK_POSITION_ABSOLUTE=2,	//absolute closeloop positioning with feedforward torque
	STEPCTRL_FEEDBACK_VELOCITY=3,			//velocity closeloop positioning with feedforward torque
	//todo replace FEEDBACK with SENSED
	STEPCTRL_FEEDBACK_TORQUE=4,				//torque control with no closeloop
	STEPCTRL_FEEDBACK_CURRENT=5,			//current control
	STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF=6,	//last torque ramp off

	//Classical sensorless openloop
	STEPCTRL_OPENLOOP_POSITION_RELATIVE=7,	//relative openloop positioning
	STEPCTRL_OPENLOOP_POSITION_ABSOLUTE=8,	//absolute openloop positioning
	STEPCTRL_OPENLOOP_VELOCITY=9,			//velocity openloop positioning
	STEPCTRL_OPENLOOP_SLOW_CURRENT_OFF=6,	//last current ramp off
} stepCtrlFeedbackMode_t;

typedef struct { //closeloop position controller
	int32_t Kp;
	int32_t Ki;
	int32_t Kd;
} PID_t;

#define DEGREES_TO_ANGLERAW(x) ( ((float)(x) / 360.0f * ANGLE_STEPS) )
#define ANGLERAW_T0_DEGREES(x) ( ((float)(x) * 360.0f / ANGLE_STEPS) )

//scales PID parameters from Flash (floating point) to int32_t used in control 
#define CTRL_PID_SCALING 			(int16_t)(8096)

#define S_to_uS   	(uint32_t)(1000000)
#define SAMPLING_PERIOD_uS	(uint16_t)(40) //sampling time in uS of control loop. 35uS puts theoretical limit of ~125rev/s on the motor which is plenty.  Adjust to reduce harmonics. 
#define SAMPLING_HZ		(uint32_t)(S_to_uS / SAMPLING_PERIOD_uS) //update rate of control loop

//api - nvram mirror
extern volatile MotorParams_t motorParams;
extern volatile SystemParams_t systemParams;

//api - control states
extern volatile bool StepperCtrl_Enabled;
extern volatile stepCtrlError_t stepCtrlError;
extern volatile bool enableFeedback;
extern volatile bool enableCloseLoop;
extern volatile bool enableSoftOff;

//api - commanded
extern volatile int32_t desiredLocation;
extern volatile int_fast16_t feedForward;
extern volatile int_fast16_t closeLoopMax;

//api - measured
extern volatile int32_t currentLocation;
extern volatile int_fast16_t closeLoop;
extern volatile int_fast16_t control;
extern volatile int32_t speed_slow;
extern volatile int32_t loopError;

void StepperCtrl_setLocationFromEncoder(void);
int32_t StepperCtrl_updateCurrentLocation(void);
uint16_t StepperCtrl_calibrateEncoder(bool update);
uint16_t CalibrationMove(bool updateFlash, int8_t dir, int32_t *microSteps, uint8_t *passes, uint16_t calLocOffset);
uint16_t StepperCtrl_sampleMeanEncoder(uint16_t numSamples);
uint16_t StepperCtrl_getEncoderAngle(void);
void StepperCtrl_updateParamsFromNVM(void);
float StepperCtrl_measureStepSize(void);
stepCtrlError_t StepperCtrl_begin(void);
void StepperCtrl_enable(bool enable);
void StepperCtrl_setMotionMode(uint8_t mode);
bool StepperCtrl_processMotion(void);
bool StepperCtrl_simpleFeedback(int32_t error);
void StepperCtrl_moveToAngle(int16_t a, uint16_t ma);


#endif
