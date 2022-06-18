/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CAN_H
#define CAN_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "board.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_TypeDef hcan;

void CAN_TransmitMotorStatus(void);
void CAN_MsgsFiltersSetup(void);
void CheckTxStatus(uint8_t TransmitMailbox);
void CAN_InterpretMesssages(CanRxMsg message);
bool Check_Control_CAN_rx_validate(void);

#define CHECK_RX_FAIL_LIM 5

#ifdef __cplusplus
}
#endif
#endif // CAN_H