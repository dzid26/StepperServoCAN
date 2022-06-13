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
  
  // uint16_t state=TLE5012_ReadState();// read state register
  // if(state & 0x0080) {// Status Magnitude Out of Limit
  //   ok = false; // GMR-magnitude out of limit
  // }
  ok = TLE5012_WriteAndCheck(WRITE_MOD2_VALUE,0x804);  //Set: ANG_Range 360 15bit, ANG_DIR: CCW, PREDICT: ON, AUTOCAL: OFF

  return ok;
}


//
uint16_t TLE5012_ReadValue(uint16_t Command)
{
  uint16_t safety;
  uint16_t data;

  TLE012_CS_L;
  SPI_TX_ON;
  data = SPI_WriteAndRead(SPI1, Command); //command loopback

  SPI_TX_OFF;
  Command = DUMMY; //dummy send used for reading
  data = SPI_WriteAndRead(SPI1, Command);
  safety = SPI_WriteAndRead(SPI1, Command);
  TLE012_CS_H;

  return data;
}

//
void TLE5012_WriteValue(uint16_t Command,uint16_t RegValue)
{
  TLE012_CS_L;
  SPI_TX_ON;
  SPI_WriteAndRead(SPI1, Command);
  SPI_WriteAndRead(SPI1, RegValue);
  TLE012_CS_H;
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
  raw_angle=(TLE5012_ReadValue(READ_ANGLE_VALUE) & DELETE_BIT_15);
  return raw_angle;
}




