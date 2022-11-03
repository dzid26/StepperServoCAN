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
/*
 * @brief  TLE5012B simple driver
  * @param  None
  * @retval None
  * @note Based on BTT implementation with corrections from datasheet. 
*/


bool TLE5012_begin(void)
{
  bool ok = true;
  
  uint16_t state=TLE5012_ReadState();// read state register
  if((state & 0x0080U) != 0U) {// Status Magnitude Out of Limit
    ok = ok && false; // GMR-magnitude out of limit
  }
  ok = ok && TLE5012_WriteAndCheck(WRITE_MOD2_VALUE, 0x804);  //Set: ANG_Range 360 15bit, ANG_DIR: CCW, PREDICT: ON, AUTOCAL: OFF
  //todo calculate CRC for crc_par register to remove S_FUSE error

  return ok;
}


//
volatile uint16_t safety;
uint16_t TLE5012_ReadValue(uint16_t Command)
{
  uint16_t data;
  
  TLE012_CS_L;
  SPI_Cmd(TLE5012B_SPI, ENABLE);
  SPI_Write(TLE5012B_SPI, Command); //command write, read to just clear the buffer
  
  //delay_us(1); //tle5012 - twr_delay  - seems like it works without the delay

  SPI_RX_ON;
  data = SPI_Read(TLE5012B_SPI);
  safety = SPI_Read(TLE5012B_SPI);
  TLE012_CS_H;
  SPI_Cmd(TLE5012B_SPI, DISABLE);
  SPI_RX_OFF;
  
  return data;
}

//
void TLE5012_WriteValue(uint16_t Command,uint16_t RegValue)
{
  TLE012_CS_L;
  SPI_Cmd(TLE5012B_SPI, ENABLE);
  SPI_Write(TLE5012B_SPI, Command);
  SPI_Write(TLE5012B_SPI, RegValue);
  TLE012_CS_H;
  SPI_Cmd(TLE5012B_SPI, DISABLE);
  // delay_us(1);

}

bool TLE5012_WriteAndCheck(uint16_t Command,uint16_t RegValue)
{
  TLE5012_WriteValue(Command,RegValue);
  uint16_t data = TLE5012_ReadValue(Command+WRITE_READ_REG_OFFSET); //read registers are offset by 0x3000
  if(data != RegValue)
  {
    return false;
  }
  return true;
}

//
uint16_t TLE5012_ReadState(void)
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




