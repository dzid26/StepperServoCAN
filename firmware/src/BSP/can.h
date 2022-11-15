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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef CAN_H
#define CAN_H

#include "stm32f10x_can.h"

extern CAN_TypeDef hcan;

void CAN_TransmitMotorStatus(uint32_t frame);
void CAN_MsgsFiltersSetup(void);
bool Check_Control_CAN_rx_validate_tick(void);

extern volatile uint32_t can_err_rx_cnt;

#endif // CAN_H