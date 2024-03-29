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

#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x_spi.h"

uint16_t SPI_WriteAndRead(SPI_TypeDef* _SPIx, uint16_t Data);
void SPI_Write(SPI_TypeDef* _SPIx, uint16_t data);
uint16_t SPI_Read(SPI_TypeDef* _SPIx);

#endif
