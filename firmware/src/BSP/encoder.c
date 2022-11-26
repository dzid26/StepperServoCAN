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

#include "encoder.h"
#include "calibration.h"
#include "tle5012.h"

bool Encoder_begin(void){
	return TLE5012_begin();
}

uint16_t ReadEncoderAngle(void){ 
	//TLE5012 is 15bits - 32767 corresponds to 360deg
	return (uint16_t)(TLE5012_ReadAngle()<<1U); //Scale (0-32767) -> (0-65535)
}

//Get oversampled encoder angle - simple averaging
uint16_t OverSampleEncoderAngle(uint16_t numSamples){
	uint32_t sum = 0;
	for(uint16_t k=0; k < numSamples; k++){
		sum += ReadEncoderAngle();
	}
	return (uint16_t)(sum/numSamples);
}