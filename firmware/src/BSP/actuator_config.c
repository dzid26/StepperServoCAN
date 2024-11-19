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

// ----- should be set by the user --------------------------------------------------------------------------------
const bool USE_VOLTAGE_CONTROL = false; // voltage or current control - voltage control recommended for hardware v0.3

// select simple or advanced parameters
// simple parameters (rated torque and current) are usually overstated by manufacturers
// use simple parameters if you don't want to measure motor_k_bemf
// or use motor_k_bemf instead to get more accurate torque and current relationship
const bool USE_SIMPLE_PARAMETERS = false;

// SIMPLE PARAMETERS:
const float motor_rated_current = (float) 1.3; // A
const float motor_rated_torque =  (float) 28;  // Ncm

// MEASURED PARAMETERS: // todo store these in NVRAM
volatile int16_t motor_k_bemf = 1360; // mV/(rev/s) -   hold F2 to measure, use MCUViewer to see new motor_k_bemf value and update manually here
// note, when motor_k_bemf is too low, the motor can have higher top speed when unloaded (unintentional field weakening via I_d), but power and torque will not be accurate
volatile int16_t phase_R = 2400;         // mOhm -      it's best to measure this
volatile int16_t phase_L = 3230;         // uH -        use datasheet value or RLC meter to measure - correct value maximizes peak motor power

// specify gearing parameters here:
const float motor_gearbox_ratio = 5.0F+(2.0F/11.0F); // gearbox ratio - enter planetary gearbox tooth calculation for best accuracy
const float final_drive_ratio = 2.0F;                // assembly gearing ratio

const int8_t anticogging_factor = 30; //minimizes cogging under load - (0-127) -value to be chosen experimentally 

// ------  end user settings --------------------------------------------------------------------------------------




float volatile gearing_ratio;
float volatile actuatorTq_to_current; // mA/Nm - (ignores gearbox efficiency)
float volatile current_to_actuatorTq; // Nm/mA - (ignores gearbox efficiency)
volatile float motor_k_torque; // Nm/A

// interprets motor parameters
void update_actuator_parameters(void){
    gearing_ratio = motor_gearbox_ratio * final_drive_ratio;

    if (USE_SIMPLE_PARAMETERS) {
        motor_k_torque = (motor_rated_torque / 100) / motor_rated_current;
        motor_k_bemf = motor_k_torque * 1000 * 2.0f * 3.1415f;
    }else{
        // motor torque and BEMF constant are directly correlated
        // this is the easiest way to get precise torque / current relationship without a load cell
        // as long as magnetic saturation doesn't not occur (usually below rated current), the relationship is simply:
        // k_torque[Nm/A] = k_bemf[V/rad/s]
        motor_k_torque = (float)motor_k_bemf / 1000 / 2 / 3.1415f;
    }
    current_to_actuatorTq = motor_k_torque / 1000 * gearing_ratio;
    actuatorTq_to_current = 1 / current_to_actuatorTq;


    closeLoopMaxDes = 2000U; // position control maximum close loop current [mA] to limit stresses and heat generation

}