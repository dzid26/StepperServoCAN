 /**
 * MKS SERVO42B
 * Copyright (c) 2020 Makerbase. 
 *
 * Based on nano_stepper project by Misfittech
 * Copyright (C) 2018  MisfitTech LLC.
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

//SPI Write and Read
uint16_t SPI_WriteAndRead(SPI_TypeDef* SPIx, uint16_t data)
{
	uint_fast16_t timeout = 0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE)) == RESET)
	{
		++timeout;  //normally this doesn't occur and timeout is 0
		if(timeout >= 400)
		{
			return 0;
		}		
	}
	SPI_I2S_SendData(SPIx, data);
	
	timeout = 0;
	while((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE)) == RESET)
		{
		++timeout;
		if(timeout >= 400)
		{
			return 0;
		}
	}
	return SPI_I2S_ReceiveData(SPIx);
}

//SPI write
bool SPI_Write(SPI_TypeDef* SPIx, uint8_t data)
{
	uint_fast16_t timeout = 0;
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) != RESET)
	{
		++timeout;
		if(timeout >= 400)
		{
			return false;		
		}
	}
	
	SPI_I2S_SendData(SPIx, data);
	
	timeout = 0;
	while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
	{
		++timeout;
		if(timeout >= 400)
		{
			return false;
		}
	}
	
	timeout = 0;
	while((SPIx->SR & SPI_I2S_FLAG_BSY) != RESET)
	{
		++timeout;
		if(timeout >= 400)
		{
			return false;		
		}
	}
	
	return true;
}
