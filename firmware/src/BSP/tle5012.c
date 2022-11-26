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

#include "tle5012.h"
#include "delay.h"
#include <stdio.h>

/*
 * @brief  TLE5012B simple driver
  * @param  None
  * @retval None
  * @note Based on BTT implementation with corrections from datasheet. 
*/



//
static volatile uint16_t safety;
static uint16_t TLE5012_ReadValue(uint16_t command)
{
  uint16_t data;

  TLE5012_ACTIVE;
  SPI_RX_OFF;
  SPI_Cmd(TLE5012B_SPI, ENABLE);
  SPI_Write(TLE5012B_SPI, command|READ_FLAG); //command write. 

	__disable_irq();
  SPI_RX_ON;
  data = SPI_Read(TLE5012B_SPI);
  //wait one SPI clock cylce and then disable SPI just before last RX
  delay_us(1); //this delay can be shorter, but since it is during next SPI word being received, it is not making any difference
  SPI_Cmd(TLE5012B_SPI, DISABLE);//this will stop SCK right right after last word is read
	__enable_irq();
  safety = SPI_Read(TLE5012B_SPI);//read last word from the buffer
  TLE5012_INACTIVE;
  
  return data;
}

//
static void TLE5012_WriteValue(uint16_t command, uint16_t regValue)
{
  TLE5012_ACTIVE;
  SPI_RX_OFF;
  SPI_Cmd(TLE5012B_SPI, ENABLE);
  SPI_Write(TLE5012B_SPI, command);
  SPI_Write(TLE5012B_SPI, regValue);
  SPI_Cmd(TLE5012B_SPI, DISABLE);
  TLE5012_INACTIVE;
}

static bool TLE5012_WriteAndCheck(uint16_t command,uint16_t regValue)
{
  TLE5012_WriteValue(command,regValue);
  uint16_t data = TLE5012_ReadValue(command);
  if(data != regValue)
  {
    return false;
  }
  return true;
}

//
static uint16_t TLE5012_ReadState(void)
{
  return (TLE5012_ReadValue(READ_STATUS));
}

//
uint16_t TLE5012_ReadAngle(void)
{
  uint16_t raw_angle;
  raw_angle=(TLE5012_ReadValue(READ_ANGLE_VALUE) & DELETE_BIT_15); //0-32767
  return raw_angle;
}


bool TLE5012_begin(void)
{
  bool ok = true;

  uint16_t state=TLE5012_ReadState();// read state register
  if(state == 0U){
    (void) printf("\nTLE5012 SPI comm error, data: %d\n", state);
    ok = false;
  }
  if((state & 0x0080U) != 0U) {// S_MAGOL - GMR Magnitude Out of Limit
    (void) printf("\nMagnet too strong, data: %d\n", state);
    ok = false;
  }
  ok = ok && TLE5012_WriteAndCheck(WRITE_MOD2_VALUE, 0x804U);  //Set: ANG_Range 360 15bit, ANG_DIR: CCW, PREDICT: ON, AUTOCAL: OFF
  //todo calculate CRC for crc_par register to remove S_FUSE error
  return ok;
}

