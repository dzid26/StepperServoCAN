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
 * Calculation of actuator coefficients.
 */

#include "actuator_config.h"
#include "stepper_controller.h"

float volatile gearing_ratio;
float volatile actuatorTq_to_current;
float volatile current_to_actuatorTq;

// todo move these parameters to NVRAM

const int16_t phase_R = 3000;         // mOhm
const int16_t phase_L = 3230;         // uH
const int16_t motor_k_bemf = 1280+100; // mV/(rev/s)
static float  motor_k_torque; //Nm/A

// specify gearing parameters here:
const float motor_gearbox_ratio = 5.0F+(2.0F/11.0F); // gearbox ratio - enter planetary gearbox tooth calculation for best accuracy
const float final_drive_ratio = 2.0F;            // assembly gearing ratio

const int8_t anticogging_factor = 30; //minimizes cogging under load - (0-127) -value to be chosen experimentally 

void update_actuator_parameters(void){
    gearing_ratio = motor_gearbox_ratio * final_drive_ratio;

    //k_torque[Nm/A] = k_bemf[V/rad/s]
    motor_k_torque = (float)motor_k_bemf / 1000.0f / 2.0f / 3.1415f;

    current_to_actuatorTq = motor_k_torque / 1000.0f * gearing_ratio;
    actuatorTq_to_current = 1.0f / current_to_actuatorTq;
    
    closeLoopMaxDes = 2000U; // posiiton control maximum close loop current [mA] to limit stresses and heat generation

}