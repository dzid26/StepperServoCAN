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
 * along with this program.  If not, see <www.gnu.org/licenses/>.
 *
 */
 
#ifndef __STEPPER_CONTROLLER_H
#define __STEPPER_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	CW_ROTATION = 0,
	CCW_ROTATION = 1,
} RotationDir_t; //sizeof(RotationDir_t)=1

typedef enum {
	ERROR_PIN_MODE_ENABLE = 0, //error pin works like enable on step sticks
	ERROR_PIN_MODE_ACTIVE_LOW_ENABLE = 1, //error pin works like enable on step sticks
} ErrorPinMode_t; //sizeof(ErrorPinMode_t)=1

typedef enum {
	CTRL_TORQUE = 0, //simple error controller
	CTRL_POS_PID =1, //PID  Position controller
	CTRL_POS_VELOCITY_PID = 2, //PID  Velocity controller
} feedbackCtrl_t; //sizeof(feedbackCtrl_t)=1


typedef enum {
	STEPCTRL_NO_ERROR=0,
	STEPCTRL_NO_POWER=1, 	//no power to motor
	STEPCTRL_NO_CAL=2,	 	//calibration not set
	STEPCTRL_NO_ENCODER=3,	//encoder not working
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

	STEPCTRL_FEEDBACK_KBEMF_ADAPT=127,		//special calibration mode

	//Classical sensorless openloop
	STEPCTRL_OPENLOOP_POSITION_RELATIVE=7,	//relative openloop positioning
	STEPCTRL_OPENLOOP_POSITION_ABSOLUTE=8,	//absolute openloop positioning
	STEPCTRL_OPENLOOP_VELOCITY=9,			//velocity openloop positioning
	STEPCTRL_OPENLOOP_SLOW_CURRENT_OFF=6,	//last current ramp off
} stepCtrlFeedbackMode_t;

typedef struct { //closeloop position controller
	int16_t Kp;
	int16_t Ki;
	int16_t Kd;
} PID_t;

extern volatile PID_t pPID; //positional current based PID control parameters
extern volatile PID_t vPID; //velocity PID control parameters

//scales PID parameters from Flash (floating point) to int32_t used in control 
#define CTRL_PID_SCALING 			(int16_t)(4096)

#define FULLSTEPS_1_8	200U  // 1.8deg stepper motor
#define FULLSTEPS_0_9	400U  // 0.9deg stepper motor
#define FULLSTEPS_NA	(invalid)


#define S_to_uS   	(uint32_t)(1000000)
#define SAMPLING_PERIOD_uS	(uint16_t)(40) //sampling time in uS of control loop. 35uS puts theoretical limit of ~125rev/s on the motor which is plenty.  Adjust to reduce harmonics. 
#define SAMPLING_HZ		(uint32_t)(S_to_uS / SAMPLING_PERIOD_uS) //update rate of control loop

//api - control states
extern volatile bool controlsEnabled;
extern volatile stepCtrlError_t stepCtrlError;
extern volatile bool enableSensored;
extern volatile bool enableCloseLoop;
extern volatile bool enableSoftOff;

//api - commanded
extern volatile int32_t desiredLocation;
extern volatile int16_t feedForward;
extern volatile int16_t closeLoopMaxDes;

//api - measured
extern volatile int32_t currentLocation;
extern volatile int16_t closeLoop;
extern volatile int16_t control;
extern volatile int16_t control_actual;
extern volatile int32_t speed_slow;
extern volatile int32_t loopError;

stepCtrlError_t StepperCtrl_begin(void);
void StepperCtrl_enable(bool enable);
void StepperCtrl_setMotionMode(uint8_t mode);
void StepperCtrl_setCurrent(int16_t current);
void StepperCtrl_setCloseLoopCurrentLim(int16_t current);
bool StepperCtrl_processMotion(void);


#endif
