/**
  ******************************************************************************
  * File Name          : CAN.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

CanRxMsg rx_message;
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_TypeDef hcan;
void CAN_MsgsFiltersSetup()
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
  }

void CAN_TransmitMyMsg(void)
{
  CanTxMsg TxMessage;
	uint32_t i = 0;
	uint8_t TransmitMailbox = 0;
	uint8_t status = 0;
  /* transmit */
  TxMessage.StdId=0x11;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.IDE=CAN_ID_STD;
  TxMessage.DLC=2;
  TxMessage.Data[0]=0xCA;
  TxMessage.Data[1]=0xFE;

  TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);

  // wait until CAN transmission is OK
  i = 0;
  while((status != CANTXOK) && (i != 0xFFFF))               
  {
    status = CAN_TransmitStatus(CAN1, TransmitMailbox);
    i++;
  }

}

uint64_t CAN_ReadPosition()
{ 
	uint8_t i;
  uint64_t pos=0;
	for(i=0; i < 7; i++)
    pos+=( ((uint64_t) rx_message.Data[i])<<((7-i)*8));
  return pos;
}

void USB_LP_CAN1_RX0_IRQHandler()
{
  CAN_Receive(CAN1, CAN_FIFO0,&rx_message);
  if(rx_message.IDE == 18){
      CAN_ReadPosition();
  }
  // xQueueSendFromISR(xCAN_receive_queue0, &RxMessage, &resch);
  // portEND_SWITCHING_ISR(resch);
}