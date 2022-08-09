/**
 * Calculation of actuator coefficients.
 * 
 */

#include "actuator_config.h"

float volatile gearing_ratio;
float volatile actuatorTq_to_current;
float volatile current_to_actuatorTq;

//todo move these parameters to NVRAM
//specify motor parameters here:
const uint16_t rated_current = 1500; //mA
const uint16_t rated_torque = 40;   //cNm

//specify gearing parameters here:
const float motor_gearbox_ratio = 5; //gearbox ratio
const float final_drive_ratio = 2; //assembly gearing ratio


void update_actuator_parameters(void){
    gearing_ratio = motor_gearbox_ratio * final_drive_ratio;

    actuatorTq_to_current = (float) rated_current / rated_torque * 100 / gearing_ratio;
    current_to_actuatorTq = 1 / actuatorTq_to_current;
}