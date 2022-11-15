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


static bool CalibrationTable_updateTableValue(uint16_t index, uint16_t value)
{
	CalData[index].value =	value;
	CalData[index].error = ANGLE_STEPS / CALIBRATION_TABLE_SIZE;
	return true;
}

bool CalibrationTable_calValid(void)
{
	uint16_t i;
	for (i=0; i < CALIBRATION_TABLE_SIZE; i++)
	{
		if (CalData[i].error == CALIBRATION_ERROR_NOT_SET)
		{
			return false;
		}
	}
	
	if (nvmFlashCalData->status != valid)
	{
		CalibrationTable_saveToFlash();
	}
	return true;
}


static uint16_t interp(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x)
{
	int32_t dx;
	int32_t dy;
	int32_t dx2;
	int32_t y;

	dx = x2 - x1;
	dy = y2 - y1;
	dx2 = x - x1;
	y = y1 + (int32_t)DIVIDE_WITH_ROUND(dx2 * dy, dx); //(y-y1)=k(x-x1), k=(y2-y1)/(x2-x1)
	if (y < 0)
	{
		y = y + ANGLE_STEPS;
	}
	if (y > ANGLE_MAX)
	{
		y = y - ANGLE_STEPS;
	}
	return (uint16_t)y;
}

uint16_t CalibrationTable_reverseLookup(uint16_t encoderAngle)
{
	uint16_t i = 0;	
	int32_t a1;
	int32_t a2;
	int32_t b1;
	int32_t b2;	
	int32_t x;
	uint16_t y;

	x = (int32_t)encoderAngle;
	if (x < ((int32_t)nvmFlashCalData->MIN))
	{
		x = x + CALIBRATION_STEPS;
	}

	i = 0;
	while(i < CALIBRATION_TABLE_SIZE)
	{
		a1 = (int32_t)CalData[i].value;

		//handle index wrap around
		if(i == (CALIBRATION_TABLE_SIZE - 1))
		{
			a2 = (int32_t)CalData[0].value;
		}else
		{
			a2 = (int32_t)CalData[i+1].value;
		}

		//wrap
		if (fastAbs(a1 - a2) > CALIBRATION_WRAP)
		{
			if (a1 < a2)
			{
				a1 = a1 + CALIBRATION_STEPS;
			}else
			{
				a2 = a2 + CALIBRATION_STEPS;
			}
		}

		//finding matching location
		if ( (x >= a1 && x <= a2) || (x >= a2 && x <= a1) )
		{
			//x1=a1  y1=b1=i*(65536/200)+0.5   x2=a2  y2=b2=(i+1)*(65536/200)+0.5   x=encoderAngle
			b1 = (int32_t)DIVIDE_WITH_ROUND((int64_t)i     * ANGLE_STEPS, CALIBRATION_TABLE_SIZE);
			b2 = (int32_t)DIVIDE_WITH_ROUND((int64_t)(i+1) * ANGLE_STEPS, CALIBRATION_TABLE_SIZE);
			y = interp(a1,b1,a2,b2,x); //y=y1+k(x-x1), k=(y2-y1)/(x2-x1)
			return y;
		}
		i++;
	}
	return 0;	//we did some thing wrong
}
uint16_t CalibrationTable_fastReverseLookup(uint16_t fastEncoderAngle)
{
	uint16_t fastLook;
	if (fastCalVaild == true)	//assume calibration is good
	{  									//we only have 16384 values in table
		fastLook = (uint16_t)(nvmFastCal->angle[fastEncoderAngle >> 1] + ((fastEncoderAngle & 0x0001) << 1)); //((fastEncoderAngle % 2) << 1)
	}else
	{
		fastLook =  CalibrationTable_reverseLookup(fastEncoderAngle);
	}
	return fastLook;//0-65535
}

static uint16_t interp2(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x)
{
	int32_t dx;
	int32_t dy;
	int32_t dx2;
	int32_t y;

	dx = x2 - x1;
	dy = y2 - y1;
	dx2 = x - x1;
	y = y1 + (int32_t)DIVIDE_WITH_ROUND(dx2 * dy, dx); //(y-y1)=k(x-x1), k=(y2-y1)/(x2-x1)

	if (y < 0)
	{
		y = y + CALIBRATION_STEPS;
	}
	if (y >= CALIBRATION_STEPS)
	{
		y = y - CALIBRATION_STEPS;
	}
	return (uint16_t)y;
}

void CalibrationTable_saveToFlash(void)
{
	uint16_t min, max;
	FlashCalData_t data;
	
	max = min = CalData[0].value;
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++ )
	{
		if(CalData[i].value < min)	{min = CalData[i].value;}
		if(CalData[i].value > max)	{max = CalData[i].value;}
		
		data.FlashCalData[i] = CalData[i].value;
	}
	data.status = valid;
	data.MIN = min;
	data.MAX = max;
	
	
	nvmWriteCalTable(&data); //CalTable

	CalibrationTable_createFastCal(); //FastCalTable
}

//FastCal writes 32kB to the end flash starting at page32
static void CalibrationTable_createFastCal(void)
{
	uint16_t i=0;
	uint16_t checkSum = 0;
	uint16_t data[FLASH_ROW_SIZE]; //1K
	uint8_t page=0;

	const uint8_t step = 2U; //we have only 32kB and 32768 uint16 angles to store

	uint16_t norm_offset = CalibrationTable_reverseLookup(0);
	
	while(i < CALIBRATION_STEPS/step) 
	{
		uint16_t x = 0;
		x = CalibrationTable_reverseLookup(i*step); //calculating fast calibration lookup every other angle int
		//CalibrationTable_fastReverseLookup() accounts for storing every other angle
		data[i % FLASH_ROW_SIZE] = x;
		
		i++;
		if ((i % FLASH_ROW_SIZE)==0U) //1k bytes = 512 uint16_t
		{
			nvmWriteFastCalTable(&data, page);
			page++;
		}
		checkSum += x;
	}
	Flash_ProgramSize(FLASH_checkSum_ADDR, &checkSum, 1);	//checkSum
	
	fastCalVaild = true;
}

static void CalibrationTable_loadFromFlash(void)
{
	uint16_t i;

	for(i=0; i < CALIBRATION_TABLE_SIZE; i++)	//Reading Calbiration from Flash
	{
		CalData[i].value = nvmFlashCalData->FlashCalData[i];
		CalData[i].error = CALIBRATION_MIN_ERROR;
	}
}

void CalibrationTable_init(void)
{
	uint16_t i;

	if(valid == nvmFlashCalData->status)
	{
		CalibrationTable_loadFromFlash();
		CalibrationTable_updateFastCal();
		
	}else
	{
		for(i=0; i < CALIBRATION_TABLE_SIZE; i++)
		{
			CalData[i].value = 0;
			CalData[i].error = CALIBRATION_ERROR_NOT_SET;
		}
	}
}

static void CalibrationTable_updateFastCal(void)
{
	uint16_t i;
	uint16_t checkSum = 0;
	bool NonZero = false;
	for(i=0; i < 16384; i++)
	{
		checkSum += nvmFastCal->angle[i];
		if(checkSum != 0)
		{
			NonZero = true;
		}
	}
	
	if(checkSum != Flash_readHalfWord(FLASH_checkSum_ADDR) || NonZero != true)
	{		
		CalibrationTable_saveToFlash();
	}
	else
	{
		fastCalVaild = true;
	}
}

//We want to linearly interpolate between calibration table angle
uint16_t CalibrationTable_getCal(uint16_t actualAngle)
{
	uint16_t indexLow;
	uint16_t indexHigh;
	int32_t x1;
	int32_t x2;
	int32_t y1;
	int32_t y2;
	uint16_t value;

	indexLow  = (uint16_t) (((uint32_t)actualAngle * CALIBRATION_TABLE_SIZE) / ANGLE_STEPS);
	indexHigh = indexLow + 1;

	x1 = ((int32_t)indexLow  * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE;
	x2 = ((int32_t)indexHigh * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE;
	
	if(indexHigh >= CALIBRATION_TABLE_SIZE)
	{
		indexHigh -= CALIBRATION_TABLE_SIZE;
	}
	y1 = CalData[indexLow].value;
	y2 = CalData[indexHigh].value;

	//handle the wrap condition
	if (fastAbs((int32_t)(y2 - y1)) > CALIBRATION_WRAP)
	{
		if (y2 < y1)
		{
			y2 = y2 + CALIBRATION_STEPS;
		}else
		{
			y1 = y1 + CALIBRATION_STEPS;
		}
	}

	value = interp2(x1, y1, x2, y2, actualAngle);

	return value;
}



// return is angle in degreesx100 ie 360.0 is returned as 36000
float StepperCtrl_measureStepSize(void)
{
	int32_t angle1,angle2,x;
	A4950_enable(true);

	uint16_t stepCurrent = motorParams.currentMa;
	/////////////////////////////////////////
	//// Measure the full step size /////
	/// Note we assume machine can take one step without issue///

	A4950_move(0, stepCurrent); //this puts stepper motor at stepAngle of zero
	delay_ms(200);

	angle1 = StepperCtrl_sampleMeanEncoder(102); //angle1
	A4950_move(A4950_STEP_MICROSTEPS/2,stepCurrent); //move one half step 'forward'
	delay_ms(100);
	A4950_move(A4950_STEP_MICROSTEPS,stepCurrent); //move one half step 'forward'
	delay_ms(200);
	angle2 = StepperCtrl_sampleMeanEncoder(102); //angle2

	if ( fastAbs(angle2 - angle1) > CALIBRATION_WRAP )
	{
		//we crossed the wrap around
		if (angle1 > angle2)
		{
			angle1 = angle1 + (int32_t)CALIBRATION_STEPS;
		}else
		{
			angle2 = angle2 + (int32_t)CALIBRATION_STEPS;
		}
	}

	// if x is ~180 we have a 1.8 degree step motor, if it is ~90 we have 0.9 degree step
	x = (int32_t)(((int64_t)(angle2 - angle1) * 36000) / (int32_t)CALIBRATION_STEPS);

	//move back
	A4950_move(A4950_STEP_MICROSTEPS/2,stepCurrent);
	delay_ms(100);
	A4950_move(0,stepCurrent);

	A4950_enable(false);

	return ((float)x)/100.0f;
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
		
		sampled = StepperCtrl_sampleMeanEncoder(202)<<1; //collect angle every half step for 1.8 stepper - returns (0-32767) scaled to (0-65535)
		
		int16_t dist = (int16_t)(sampled - cal);

		
		if (updateFlash && !preRun) {
			uint16_t average = cal + (dist / (*passes)); //add half a distance to average
			CalibrationTable_updateTableValue(( (dir<0) ? (preRunSteps + CALIBRATION_TABLE_SIZE -j) : j)%CALIBRATION_TABLE_SIZE, average>>1);	// (0-65535) scaled to (0-32767)
		}
		//move one half step at a time, a full step move could cause a move backwards
	
		dist = fastAbs(dist>>1);
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
	uint16_t calZeroOffset=(StepperCtrl_sampleMeanEncoder(202)<<1) - (CalibrationTable_getCal(0)<<1);//returns (0-32767) scaled to (0-65535)
	
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
