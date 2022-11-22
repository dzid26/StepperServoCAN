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
 
#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include <stdbool.h>

#define ANGLE_STEPS 						((uint32_t)65536U) 
#define ANGLE_MAX 							((uint16_t)65535U)
#define ANGLE_WRAP 							((int32_t)32768)

#define DEGREES_TO_ANGLERAW(x) ( ((float)(x) / 360.0f * (float)ANGLE_STEPS) )
#define ANGLERAW_T0_DEGREES(x) ( ((float)(x) * 360.0f / (float)ANGLE_STEPS) )

bool Encoder_begin(void);
uint16_t ReadEncoderAngle(void);
uint16_t OverSampleEncoderAngle(uint16_t numSamples);

#endif