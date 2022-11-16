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

bool Encoder_begin(void){
	return TLE5012_begin();
}

uint16_t ReadEncoderAngle(void){ 
	//TLE5012 is 15bits - 32767 corresponds to 360deg
	return (uint16_t)(TLE5012_ReadAngle()<<1U); //Scale (0-32767) -> (0-65535)
}

//Get oversampled encoder angle
uint16_t OverSampleEncoderAngle(uint16_t numSamples){
	uint32_t min = 0,max = 0;
	//recursive mean algorithm
	uint32_t mean = 0;
	for(int16_t k=1; k <= (int16_t)numSamples; k++){//multiple lines to satisfy MISRA
		uint32_t x = ReadEncoderAngle()<<16U; 		//bump all values into uint32 range for accuracy when dividing by k
		uint32_t delta = x - mean; 					//utlizes uint32 wrap around to automatically handle values roll over
		int32_t delta_weighted = (int32_t)delta/k; 	//cast to signed to handle division properly
		mean = mean + (uint32_t)delta_weighted;		//go back to unsigned to handle wrap around

		
		if(k == 1){
			min = x;
			max = x;
		}

		if (((int32_t)(x-max))>0){//utlizes uint32 wrap around to automatically handle values roll over
			max = x;
		}
		if (((int32_t)(x-min))<0){//utlizes uint32 wrap around to automatically handle values roll over
			min = x;
		}
	}
	
	mean = mean - (uint32_t)((int32_t)(max - mean)/(int16_t)numSamples);
	mean = mean - (uint32_t)((int32_t)(min - mean)/(int16_t)(numSamples-1U));
	
	return mean>>16U; //(0-65535)
}