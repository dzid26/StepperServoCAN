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

#include "delay.h"

// SystemCoreClock/1000000     	1us
// SystemCoreClock/100000		10us
// SystemCoreClock/1000			1ms

void delay_us (uint32_t us) {
	SysTick_Config (SystemCoreClock / 1000000U); //1us

	for(uint32_t i = 0U; i < us; ++i) {
		while(((SysTick -> CTRL) & SysTick_CTRL_COUNTFLAG_Msk) == 0U);
	}
	//disable SysTick timer
	SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void delay_ms (uint32_t ms) {
	SysTick_Config (SystemCoreClock / 1000U); //1ms
	for(uint32_t i = 0U; i < ms; ++i) {
		uint16_t t0 = 0U;
		while(((SysTick -> CTRL) & SysTick_CTRL_COUNTFLAG_Msk) == 0U){
			++t0;
			if(t0 > 20000U) { //1.3ms
				break;
			}
		}
	}
	//disable SysTick timer
	SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;  
}
