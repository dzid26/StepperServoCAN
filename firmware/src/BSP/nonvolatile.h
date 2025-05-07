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
#include "stepper_controller.h"
#include "flash.h"

typedef struct {
	uint16_t fw_version;
	RotationDir_t dirRotation;  //stores rotation direction
	uint8_t reserved1;
	uint16_t errorLimit;    //error limit before error pin asserts 65536==360degrees
	uint16_t reserved2;
	ErrorPinMode_t errorPinMode;  //is error pin used for enable, error, or bidirectional
	feedbackCtrl_t controllerMode; //feedback mode for the controller
	uint16_t reserved3; // it's invalid when eeprom is zeroed
} SystemParams_t; //sizeof(SystemParams_t)=12

typedef struct {
	uint16_t reserved1;
	uint16_t reserved2;
	bool     invertedPhase;			// motor rotating in opposite direction to angle sensor
	uint8_t  reserved3;
	uint16_t fullStepsPerRotation; //how many full steps per rotation is the motor
} MotorParams_t; //sizeof(MotorParams_t)=8

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PIDparams_t; //2xsizeof(PIDparams_t)=12

typedef struct {
	uint16_t cmdId;
	uint16_t statusID;
	uint32_t reserved1;
	uint32_t reserved2;
} CANparams_t;

#pragma pack(2) //removes 2byte padding between motorParams and pPid - this is mostly for back compatibility at this point
typedef struct {
	SystemParams_t 	systemParams;
	MotorParams_t 	motorParams;
	uint16_t parametersValid;
	PIDparams_t 	pPID; //simple PID parameters
	PIDparams_t 	vPID; //position PID parameters
	CANparams_t 	can;
} nvm_t; //sizeof(nvm_t)=58
#pragma pack()

#define PARAMETERS_FLASH_ADDR  		FLASH_PAGE62_ADDR
#define CALIBRATION_FLASH_ADDR  	FLASH_PAGE63_ADDR

// cppcheck-suppress-macro  misra-c2012-11.4 - loading values from mapped flash structure
#define nvmFlashCalData				((FlashCalData_t*)CALIBRATION_FLASH_ADDR)


//this is for wear leveling - sizeof(nvm_t) + 4bytes gap = 62 
#define NONVOLATILE_STEPS			((uint32_t)62)		//! don't change, to maintain backward compatibility
#define	valid						(uint16_t)0x0001
#define invalid						(uint16_t)0xffff  // ffs are default value for unused space

// nvram mirror
extern nvm_t nvmMirror;
extern volatile uint32_t NVM_startAddress;

extern volatile SystemParams_t liveSystemParams;
extern volatile MotorParams_t liveMotorParams;

void nonvolatile_begin(void);
void nvmWriteCalTable(void *ptrData);
void nvmWriteConfParms(void);
void validateAndInitNVMParams(void);

#endif
