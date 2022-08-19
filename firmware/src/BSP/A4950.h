 /**
 * MKS SERVO42B
 * Copyright (c) 2020 Makerbase. 
 *
 * Based on nano_stepper project by Misfittech
 * Copyright (C) 2018  MisfitTech LLC.
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
#ifndef __A4950_H
#define __A4950_H

#include <stdint.h>
#include <stdbool.h>

#include "sine.h"

#define I_RS_A4950_div     (uint16_t) (1000/10) //mOhm to Ohm and 10x multiplier
#define I_RS_A4950_rat     (uint16_t) (RS_A4950/I_RS_A4950_div) //mOhm to Ohm and 10x multiplier

#define A4950_STEP_MICROSTEPS (uint16_t) 256
//VREF_SCALER reduces PWM resolution by 2^VREF_SCALER but increases PWM freqency by 2^(VREF_SCALER-1)
#define VREF_SCALER	6
#define VREF_SINE_RATIO	(1<<VREF_SCALER)

void A4950_enable(bool enable);
int32_t A4950_move(uint16_t stepAngle, uint16_t mA);
void A4954_begin(void);

#endif
