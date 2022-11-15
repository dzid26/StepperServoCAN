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

#include "encoder.h"
#include "calibration.h"

uint16_t ReadEncoderAngle(void){ 
	//Expects 15bits - 32767 is 360deg
	return TLE5012_ReadAngle(); //(0-32767)
}

bool Encoder_begin(void){
	return TLE5012_begin();
}


uint16_t StepperCtrl_getEncoderAngle(void)
{
	uint16_t EncoderAngle;//(0-32767)

	EncoderAngle = CalibrationTable_fastReverseLookup(ReadEncoderAngle());

	return EncoderAngle;  //0-65535
}

// when sampling the mean of encoder if we are on roll over
// edge we can have an issue so we have this function
// to do the mean correctly
uint16_t StepperCtrl_sampleMeanEncoder(uint16_t numSamples)
{
	uint16_t i;
	int32_t lastx = 0,x = 0;
	int32_t min = 0,max = 0;
	int64_t sum = 0;
	int32_t mean = 0;

	for(i=0; i < numSamples; i++)
	{
		lastx = x;
		x = (int32_t)ReadEncoderAngle();
		if(i == 0)
		{
			lastx = x;
			min = x;
			max = x;
		}

		//wrap
		if (fastAbs(lastx - x) > CALIBRATION_WRAP) //2^15-1 = 32767(max)
		{
			if (lastx > x)
			{
				x = x + CALIBRATION_STEPS;
			} else
			{
				x = x - CALIBRATION_STEPS;
			}
		}

		if (x > max)
		{
			max = x;
		}
		if (x < min)
		{
			min = x;
		}

		sum = sum + x;
	}

	mean = (int32_t)(sum - min - max) / (numSamples - 2); //remove the min and the max.

	//mean 0~32767
	if(mean >= CALIBRATION_STEPS)
	{
		mean = (mean - (int32_t)CALIBRATION_STEPS);
	}
	if(mean < 0)
	{
		mean = (mean + (int32_t)CALIBRATION_STEPS);
	}

	return ((uint16_t)mean); //(0-32767)
}
