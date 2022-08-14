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
#ifndef __A4950_H
#define __A4950_H

#include <stdint.h>
#include <stdbool.h>


#define A4950_STEP_MICROSTEPS (uint16_t) 256U //Full step electrical angle
//VREF_SCALER reduces PWM resolution by 2^VREF_SCALER but increases PWM freqency by 2^(VREF_SCALER-1)
#define VREF_SCALER	6
#define VREF_SINE_RATIO	(1<<VREF_SCALER)
#define PWM_SCALER	3 //low vibration

void A4950_enable(bool enable);
void A4950_move(uint16_t stepAngle, uint16_t mA);
void A4954_begin(void);

extern volatile bool A4950_Enabled;

#endif
