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

#ifndef __FLASH_H
#define __FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define FLASH_WAIT_TIMEOUT			100000U
#define FLASH_PAGE_SIZE   			1024U
#define FLASH_ROW_SIZE   			512U

#define	FLASH_PAGE62_ADDR				0x0800F800U //Params
#define	FLASH_PAGE63_ADDR				0x0800FC00U //CalTable
#define FLASH_checkSum_ADDR			    0x0800FFFCU

#define	valid								(uint16_t)0x0001
#define invalid								(uint16_t)0xffff

void Flash_ProgramPage(uint32_t flashAddr, uint16_t* ptrData, uint16_t size);
void Flash_ProgramSize(uint32_t flashAddr, uint16_t* ptrData, uint16_t size);
uint16_t Flash_readHalfWord(uint32_t address);
uint32_t Flash_readWord(uint32_t address);

#endif
