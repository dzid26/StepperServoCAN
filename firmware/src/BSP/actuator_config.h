/**
 * StepperServoCAN
 *
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


/**
 * @ Description:
 * Quick calculation of actuator coefficients.
 */

#ifndef ACTUATOR_CONFIG_H
#define ACTUATOR_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

extern const bool USE_SIMPLE_PARAMETERS;
extern const bool USE_VOLTAGE_CONTROL;

extern volatile int16_t phase_R; //mOhm
extern volatile int16_t phase_L; //uH
extern volatile int16_t motor_k_bemf; // mV/(rev/s)

//calculate actuator parameters to be used by control_api 
extern volatile float gearing_ratio;
extern volatile float actuatorTq_to_current;
extern volatile float current_to_actuatorTq;

extern const int8_t anticogging_factor;

void update_actuator_parameters(bool use_simple_params);


#endif // ACTUATOR_CONFIG_H
