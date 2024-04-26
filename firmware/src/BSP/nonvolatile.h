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
 
#ifndef __NONVOLATILE_H
#define __NONVOLATILE_H

#include <stdint.h>
#include <stdbool.h>
#include "calibration.h"
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

#pragma pack(4)
typedef struct {
	__attribute__((__aligned__(4))) float Kp;
	__attribute__((__aligned__(4))) float Ki;
	__attribute__((__aligned__(4))) float Kd;
} PIDparams_t;

#pragma pack(2)
typedef struct {
	__attribute__((__aligned__(2))) uint16_t currentMa;   //maximum current for the motor
	__attribute__((__aligned__(2))) uint16_t currentHoldMa; //hold current for the motor
	__attribute__((__aligned__(2))) bool motorWiring;  //forward wiring of motor or reverse
	__attribute__((__aligned__(2))) uint16_t fullStepsPerRotation; //how many full steps per rotation is the motor
	__attribute__((__aligned__(2))) uint16_t parametersValid;
} MotorParams_t; //sizeof(MotorParams_t)=14

#pragma pack(2)
typedef struct {
	__attribute__((__aligned__(2))) uint16_t microsteps;    //number of microsteps on the dir/step pin interface from host
	__attribute__((__aligned__(2))) RotationDir_t dirRotation;  //stores rotation direction
	__attribute__((__aligned__(2))) uint16_t errorLimit;    //error limit before error pin asserts 65536==360degrees
	__attribute__((__aligned__(2))) ErrorPinMode_t errorPinMode;  //is error pin used for enable, error, or bidirectional
	__attribute__((__aligned__(2))) feedbackCtrl_t controllerMode; //feedback mode for the controller
	__attribute__((__aligned__(2))) uint16_t parametersValid;
} SystemParams_t; //sizeof(SystemParams_t)=18

typedef struct {
	__attribute__((__aligned__(4))) uint32_t reserved1;
	__attribute__((__aligned__(4))) uint32_t reserved2;
	__attribute__((__aligned__(4))) uint32_t reserved3;
} Reserved_t;
typedef struct {
	SystemParams_t 	systemParams;
	MotorParams_t 	motorParams;
	PIDparams_t 	pPID; //simple PID parameters
	PIDparams_t 	vPID; //position PID parameters
	Reserved_t 		reserved;
} nvm_t;

#define PARAMETERS_FLASH_ADDR  		FLASH_PAGE62_ADDR
#define CALIBRATION_FLASH_ADDR  	FLASH_PAGE63_ADDR

#define NVM										((nvm_t*)NVM_address)
#define nvmFlashCalData				((FlashCalData_t*)CALIBRATION_FLASH_ADDR)

//this is for wear leveling
#define NONVOLATILE_STEPS				((uint32_t)62)		//2bytes gap + sizeof(nvm_t) = 60


// nvram mirror
extern volatile uint32_t NVM_address;

extern nvm_t nvmParams;
extern volatile SystemParams_t liveSystemParams;
extern volatile MotorParams_t liveMotorParams;

void nonvolatile_begin(void);
void nvmWriteCalTable(void *ptrData);
void nvmWriteConfParms(nvm_t* ptrNVM);
void validateAndInitNVMParams(void);

#endif
