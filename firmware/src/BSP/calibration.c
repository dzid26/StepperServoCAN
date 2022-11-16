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
 
#include "calibration.h"
#include "nonvolatile.h"
#include "sine.h"
#include "A4950.h"
#include "encoder.h"
#include "delay.h"
#include "actuator_config.h"

extern volatile int32_t currentLocation;

static void CalibrationTable_createFastCal(void);
static void CalibrationTable_loadFromFlash(void);
static void CalibrationTable_updateFastCal(void);

static volatile CalData_t CalData[CALIBRATION_TABLE_SIZE];
static volatile bool	fastCalVaild = false;


static bool CalibrationTable_updateTableValue(uint16_t index, uint16_t value){
	CalData[index].value =	value;
	CalData[index].error = ANGLE_STEPS / CALIBRATION_TABLE_SIZE;
	return true;
}

bool CalibrationTable_calValid(void){
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++){
		if (CalData[i].error == CALIBRATION_ERROR_NOT_SET){
			return false;
		}
	}
	
	if (nvmFlashCalData->status != valid){ // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		CalibrationTable_saveToFlash();
	}
	return true;
}


static uint16_t interp(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x){//(0-65535)
	uint16_t dx;
	uint16_t dy;
	uint16_t dx2;
	uint16_t y;

	dx = x2 - x1;
	dy = y2 - y1;
	dx2 = x - x1;
	y = y1 + DIVIDE_WITH_ROUND(dx2 * dy, dx); //(y-y1)=k(x-x1), k=(y2-y1)/(x2-x1)
	return y;
}

static uint16_t CalibrationTable_reverseLookup(uint16_t encoderAngle){
	uint16_t x = encoderAngle;//(0-65535)
	for(uint16_t idx1 = 0; idx1 < CALIBRATION_TABLE_SIZE; idx1++){
		uint16_t idx2 = (idx1 + 1U)%CALIBRATION_TABLE_SIZE;
		uint16_t x1 = CalData[idx1].value;//(0-65535)
		uint16_t x2 = CalData[idx2].value;//(0-65535)

		//finding matching location
		uint16_t x1_x = x - x1;  //Utilize wrap around effect of subtraction to deal of with x2<x1 on calibration table wrap
		uint16_t x_x2 = x2 - x;
		if (((int16_t)x1_x >= 0) && ((int16_t)x_x2 > 0)){ //search angle within the range.
			//x1=a1  y1=b1=i*(65536/200)+0.5   x2=a2  y2=b2=(i+1)*(65536/200)+0.5   x=encoderAngle
			uint16_t y1 = (uint16_t)((uint32_t) idx1 * ANGLE_STEPS / CALIBRATION_TABLE_SIZE);
			uint16_t y2 = (uint16_t)((uint32_t) idx2 * ANGLE_STEPS / CALIBRATION_TABLE_SIZE);
			uint16_t y = interp(x1,y1,x2,y2,x); //y=y1+k(x-x1), k=(y2-y1)/(x2-x1)
			return y;//(0-65535)
		}
	}
	return 0;	//we did some thing wrong
}

static const uint16_t fastcal_angle_step = (uint16_t)(ANGLE_STEPS/FAST_CAL_TABLE_SIZE); //effectively stores 2^14 angles, which can be further interpolated to 15bit at runtime
uint16_t GetCorrectedAngle(uint16_t encoderAngle){ //(0-65535)
	uint16_t fastLook;
	if (fastCalVaild == true){	//assume calibration is good
		uint16_t fastLookIdx = encoderAngle / fastcal_angle_step; //fast lookup doesn't store all angles
		uint16_t angleRemainder = encoderAngle - (fastLookIdx * fastcal_angle_step);//improve accuracy by adding remaining bits to fast lookup
		fastLook = nvmFastCal->angle[fastLookIdx] + angleRemainder; // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
	}else{
		fastLook =  CalibrationTable_reverseLookup(encoderAngle);
	}
	return fastLook;//0-65535
}

void CalibrationTable_saveToFlash(void){
	FlashCalData_t data;
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++ ){
		data.FlashCalData[i] = CalData[i].value;
	}
	data.status = valid;
	
	nvmWriteCalTable(&data); //CalTable

	CalibrationTable_createFastCal(); //FastCalTable
}

//FastCal writes 32kB to the end flash starting at page32
static void CalibrationTable_createFastCal(void){
	uint16_t checkSum = 0;
	uint16_t data[FLASH_ROW_SIZE]; //1K
	uint8_t page=0;
	

	uint16_t norm_offset = CalibrationTable_reverseLookup(0);

	uint16_t angle=0;
	for(uint16_t i=0; i < FAST_CAL_TABLE_SIZE; i++){
		uint16_t x = CalibrationTable_reverseLookup(angle); //calculating fast calibration lookup every other angle int
		angle += fastcal_angle_step;
		data[i % FLASH_ROW_SIZE] = x;
		checkSum += x;
		
		if (((i+1U) % FLASH_ROW_SIZE)==0U){ //1k bytes = 512 uint16_t
			nvmWriteFastCalTable(&data, page);
			page++;
		}
	}
	Flash_ProgramSize(FLASH_checkSum_ADDR, &checkSum, 1);	//checkSum
	
	fastCalVaild = true;
}

//Reading Calibration from Flash
static void CalibrationTable_loadFromFlash(void){
	for(uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++){
		CalData[i].value = nvmFlashCalData->FlashCalData[i]; // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		CalData[i].error = CALIBRATION_MIN_ERROR;
	}
}

void CalibrationTable_init(void){
	if(valid == nvmFlashCalData->status){  // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		CalibrationTable_loadFromFlash();
		CalibrationTable_updateFastCal();
		
	}else{
		for(uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++){
			CalData[i].value = 0;
			CalData[i].error = CALIBRATION_ERROR_NOT_SET;
		}
	}
}

static void CalibrationTable_updateFastCal(void){
	uint16_t checkSum = 0;
	bool nonZero = false;
	for(uint16_t i=0; i < FAST_CAL_TABLE_SIZE; i++)	{
		checkSum += nvmFastCal->angle[i]; // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		if(checkSum != 0U)
		{
			nonZero = true;
		}
	}
	
	if((checkSum != Flash_readHalfWord(FLASH_checkSum_ADDR)) || (nonZero != true)){
		CalibrationTable_saveToFlash();
	}
	else{
		fastCalVaild = true;
	}
}

//We want to linearly interpolate between calibration table angle
uint16_t CalibrationTable_getCal(uint16_t actualAngle){ //actualAngle - (0-65535)
	uint16_t index;
	index = (uint16_t) (((uint32_t)actualAngle * CALIBRATION_TABLE_SIZE) / ANGLE_STEPS);
	uint16_t x1 = (uint16_t) (((uint32_t)index * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE);
	uint16_t y1 = CalData[index].value;
	
	index = (index+1U) % CALIBRATION_TABLE_SIZE;
	uint16_t x2 = (uint16_t) (((uint32_t)index * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE);
	uint16_t y2 = CalData[index].value;
	
	uint16_t value = interp(x1, y1, x2, y2, actualAngle);

	return value;//(0-65535)
}


// return is angle in degreesx100 ie 360.0 is returned as 36000
float StepperCtrl_measureStepSize(void){
	uint16_t angle1;
	uint16_t angle2;
	A4950_enable(true);

	uint16_t stepCurrent = motorParams.currentMa;
	// Measure the full step size
	// Note we assume machine can take one step without issue///
	A4950_move(0, stepCurrent); //fix the stepper
	delay_ms(200);
	angle1 = OverSampleEncoderAngle(100U); //angle1 - (0-65535)
	
	A4950_move(A4950_STEP_MICROSTEPS, stepCurrent); //move one step 'forward'
	delay_ms(200);
	angle2 = OverSampleEncoderAngle(100U); //angle2 - (0-65535)

	// delta can be about 1.8 or 0.9 degree depending on the motor
	uint16_t angle_delta_unsigned = angle2 - angle1; //makes use of circular integer wrap around to subtract any any angles
	int16_t angle_delta = (int16_t) angle_delta_unsigned;
	float deg_delta = ANGLERAW_T0_DEGREES(angle_delta);
	
	//move back
	A4950_move(0,stepCurrent);
	A4950_enable(false);

	return deg_delta;
}



//The encoder needs to be calibrated to the motor.
// we will assume full step detents are correct,
// ex 1.8 degree motor will have 200 steps for 360 degrees.
// We also need to calibrate the phasing of the motor
// to the A4950. This requires that the A4950 "step angle" of
// zero is the first entry in the calibration table.
static uint16_t CalibrationMove(bool updateFlash, int8_t dir, int32_t *microSteps, uint8_t *passes, uint16_t calZeroOffset){
	
	uint16_t maxError = 0;
	uint16_t stepCurrent = motorParams.currentMa;
	
	(*passes)++;
	int16_t microStep = A4950_STEP_MICROSTEPS*motorParams.fullStepsPerRotation/CALIBRATION_TABLE_SIZE;
	
	const uint16_t preRunSteps = CALIBRATION_TABLE_SIZE/2; //do half rotation preRun to start calibration with max hysteresis
	for (uint16_t j = 0; j < preRunSteps + CALIBRATION_TABLE_SIZE; j++) //Starting calibration 
	{
		static volatile uint16_t desiredAngle;
		static volatile uint16_t sampled;

		bool preRun = (j < preRunSteps); //rotate some to stabilize hysteresis before starting actual calibration
		delay_ms(1);
		if (updateFlash && !preRun) 
			delay_ms(100);

		
		desiredAngle = (uint16_t) DIVIDE_WITH_ROUND(*microSteps * (int32_t)(ANGLE_STEPS>>2) / A4950_STEP_MICROSTEPS, motorParams.fullStepsPerRotation>>2);
		uint16_t cal = (CalibrationTable_getCal(desiredAngle)<<1) + calZeroOffset; //returns (0-32767) scaled to (0-65535)
		
		sampled = OverSampleEncoderAngle(200U); //collect angle every half step for 1.8 stepper
		
		int16_t dist = (int16_t)(sampled - cal);

		
		if (updateFlash && !preRun) {
			uint16_t average = cal + (dist / (*passes)); //add half a distance to average
			CalibrationTable_updateTableValue(( (dir<0) ? (preRunSteps + CALIBRATION_TABLE_SIZE -j) : j)%CALIBRATION_TABLE_SIZE, average);
		}
		//move one half step at a time, a full step move could cause a move backwards
	
		dist = (int16_t) fastAbs(dist);
		if(dist > maxError && !preRun)
				{
					maxError = dist;
				}
		if(microStep > A4950_STEP_MICROSTEPS)  //for 0.9deg stepper collect data only every full step
		{
			A4950_move(*microSteps + A4950_STEP_MICROSTEPS, stepCurrent);
			delay_ms(20);
		}
		*microSteps += microStep*dir;
		A4950_move(*microSteps, stepCurrent);
	}
	return maxError;
}

uint16_t StepperCtrl_calibrateEncoder(bool updateFlash)
{
	uint16_t maxError;
	int32_t microSteps = 0;
	uint8_t passes = 0;

	A4950_enable(true);
	currentLocation = 0;

	A4950_move(0, motorParams.currentMa);
	delay_ms(50);
	uint16_t calZeroOffset=OverSampleEncoderAngle(200U) - CalibrationTable_getCal(0);//0-65535
	
	//! first calibration pass to the right
	maxError = CalibrationMove(updateFlash, 1, &microSteps, &passes, calZeroOffset); //run clockwise
	//! second calibration to the left - reduces influence of magnetic hysteresis
	delay_ms(1000);  	//give some time before motor starts to move the other direction
	if(updateFlash) {	//second pass anticlockwise when calibrating
		calZeroOffset=0; //on the second run, the calTab values are alligned with the sensor, so no offset
		maxError = CalibrationMove(updateFlash, -1, &microSteps, &passes, calZeroOffset); 

		CalibrationTable_saveToFlash(); //saves the calibration to flash
	}
	//measure new starting point
	A4950_move(0, 0); //release motor

	return maxError;
}
