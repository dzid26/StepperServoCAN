/**
 * @ Description:
 * Quick calculation of actuator coefficients.
 */

#ifndef ACTUATOR_CONFIG_H
#define ACTUATOR_CONFIG_H

#include <stdint.h>

extern const uint16_t rated_current; //mA
extern const uint16_t rated_torque; //cNm

//specify gearing parameters here:
extern const float motor_gearing_ratio; //gearbox ratio
extern const float final_drive_ratio; //assembly gearing ratio

//calculate actuator parameters to be used by control_api 
extern volatile float gearing_ratio;
extern volatile float actuatorTq_to_current;
extern volatile float current_to_actuatorTq;
void update_actuator_parameters(void);


#endif // ACTUATOR_CONFIG_H
