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

#ifndef __SINE_H
#define __SINE_H

#include <stdint.h>

#define SINE_STEPS	((uint16_t)1024)
#define SINE_PI     ((uint16_t)256)
#define SINE_MAX	((uint16_t)32768)

int16_t sine(uint16_t angle);
int16_t cosine(uint16_t angle);
int16_t sine_ripple(uint16_t angle, int8_t strength);
int16_t cosine_ripple(uint16_t angle, int8_t strength);

#endif
