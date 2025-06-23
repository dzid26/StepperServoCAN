/**
 * StepperServoCAN
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

#ifndef CAN_H
#define CAN_H

#include "stm32f10x_can.h"

extern CAN_TypeDef hcan;

void CAN_TransmitMotorStatus(uint32_t frame);
void CAN_Setup(void);
bool CAN_rx_validate_tick(void);

#define CHECK_RX_FAIL_LIM 5U
extern volatile uint8_t rx_fail_cnt;

#endif // CAN_H