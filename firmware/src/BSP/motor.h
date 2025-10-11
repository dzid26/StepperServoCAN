#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "A4950.h"

#define FULLSTEP_ELECTRIC_ANGLE (uint16_t) 256U //Full step electrical angle
#define MAX_CURRENT I_MAX_A4950
void openloop_step(uint16_t elecAngleStep, uint16_t curr_tar);
void field_oriented_control(int16_t current_target);
void base_speed_test(int16_t dir);

int16_t get_torque(void);

#endif // MOTOR_H_