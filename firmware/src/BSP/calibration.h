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
#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include "flash.h"

//changing this requires recalibration
#define	CALIBRATION_TABLE_SIZE			50U  // 50 is enough, 100, 200 also good

#define CALIBRATION_ERROR_NOT_SET (-1) //indicated that the calibration value is not set.
#define CALIBRATION_MIN_ERROR (2)  //the minimal expected error on our calibration 4 ~=+/0.2 degrees
#define CALIBRATION_MAX_ERROR (546)  //the maximal expected error on calibration 546 = 3deg

typedef struct {
	uint16_t FlashCalData[CALIBRATION_TABLE_SIZE];
	uint16_t status;
} FlashCalData_t;

typedef struct {
  uint16_t value;  //cal value
  int16_t error; 	 //error assuming it is constantly updated
} CalData_t;

void StepperCtrl_setLocationFromEncoder(void);
uint16_t StepperCtrl_calibrateEncoder(bool update);
float StepperCtrl_measureStepSize(void);
bool CalibrationTable_calValid(void);
uint16_t GetCorrectedAngle(uint16_t fastEncoderAngle);
int CalibrationTable_getValue(uint16_t actualAngle, CalData_t *ptrData);
void CalibrationTable_saveToFlash(void);
void CalibrationTable_init(void);

#endif