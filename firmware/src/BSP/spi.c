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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "spi.h"

//SPI Write and Read single word
uint16_t SPI_WriteAndRead(SPI_TypeDef* SPIx, uint16_t data)
{
	uint_fast16_t timeout = 0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)) == RESET)
	{
		++timeout;  //normally this doesn't occur and timeout is 0
		if(timeout >= 400U){
			return 0;
		}		
	}
	SPI_I2S_SendData(SPIx, data);
	
	timeout = 0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)) == RESET)
		{
		++timeout;
		if(timeout >= 400U){
			return 0;
		}
	}
	return SPI_I2S_ReceiveData(SPIx);
}

//SPI write word - transmit only mode
void SPI_Write(SPI_TypeDef* SPIx, uint16_t data)
{
	uint_fast16_t timeout = 0;
	SPI_I2S_SendData(SPIx, data);

	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
		++timeout;
		if(timeout >= 400U)
		{
			return;
		}
	}
	
	timeout = 0;
	while((SPIx->SR & SPI_I2S_FLAG_BSY) != RESET)
	{
		++timeout;
		if(timeout >= 400U)
		{
			return;		
		}
	}
	
	return;
}

//SPI Read word - receive only mode
uint16_t SPI_Read(SPI_TypeDef* SPIx)
{
	uint_fast16_t timeout = 0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)) == RESET)
		{
		++timeout;
		if(timeout >= 400U){
			return 0;
		}
	}
	return SPI_I2S_ReceiveData(SPIx);
}