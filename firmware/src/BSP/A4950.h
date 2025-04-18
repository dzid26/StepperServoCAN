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

//VREF_SCALER reduces PWM resolution by 2^VREF_SCALER but increases PWM freqency by 2^(VREF_SCALER-1)
#define VREF_SCALER	6U
#define PWM_SCALER	4U // lower numbers are audible (voltage control mode)
#define SYS_Vin 14500U //mV
#define V_TO_mV 1000

#define PHASE_LEAD_MAX_SPEED  100u //revs/s
extern const uint16_t dacPhaseLead[PHASE_LEAD_MAX_SPEED];

#define I_RS_A4950_div    10U  // div for A4950
#define I_MAX_A4950       3300 //mA

#define BODY_DIODE_DROP_mV 430U //  intrinsic body diode voltage drop - (AT8236 has 495mV)

void A4950_enable(bool enable);
void phase_current_command(int16_t I_a, int16_t I_b);
void phase_voltage_command(int16_t U_a, int16_t U_b, uint16_t curr_lim);

extern volatile bool driverEnabled;

#endif
