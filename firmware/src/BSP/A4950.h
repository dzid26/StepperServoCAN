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
#define VREF_SCALER	6U
#define PWM_SCALER	3U //low vibration
#define SYS_Vin 14500U //mV
#define V_TO_mV 1000

#define PHASE_LEAD_MAX_SPEED  250u //revs/s
extern const uint16_t dacPhaseLead[PHASE_LEAD_MAX_SPEED];

#define I_MAX_A4950       (3300) //mA

void A4950_enable(bool enable);
void apply_current_command(uint16_t elecAngleStep, uint16_t curr_tar);
void apply_volt_command(uint16_t elecAngle, int32_t U_q, int32_t U_d, uint16_t curr_lim);
void A4954_begin(void);

extern volatile bool driverEnabled;

#endif
