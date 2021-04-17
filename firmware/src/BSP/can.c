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
#include "stepper_controller.h"
#include "control_api.h"

volatile uint32_t can_rx_cnt = 0;
volatile uint32_t can_tx_cnt = 0;
volatile uint32_t can_err_cnt = 0;
volatile uint32_t can_overflow_cnt = 0;

static int GetTemperature()
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)!=SET);
  int ADCConvertedValue = ADC_GetConversionValue(ADC1);
  float fTemp = (1.6f - ADCConvertedValue*3.3f/4096)*1000/4.1f + 25;

return (int)(fTemp);

}


void CAN_MsgsFiltersSetup()
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; //CAN_FilterMode_IdList
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
  }

void CAN_TransmitMyMsg(void)
{
  CanTxMsg TxMessage;
	uint32_t i = 0;
	uint8_t TransmitMailbox = 0;
	uint8_t status = CAN_TxStatus_Failed;
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
  while((status != CANTXOK))               
  {
    status = CAN_TransmitStatus(CAN1, TransmitMailbox);
    i++;
    delay_us(1);
    if (status == CAN_TxStatus_Failed || (i == 0xFF)){
      can_err_cnt++;
      CAN_GetLastErrorCode(CAN1);
      return;
    }
  }
  can_tx_cnt++;
}

void CAN_InterpretMesssages(CanRxMsg message) { 
  switch (message.StdId){
  	case 18: {
      uint8_t i;
      uint64_t pos=0;
      for(i=0; i < 8; i++)
        pos+=(((uint64_t) message.Data[i])<<((7-i)*8));
      StepperCtrl_setDesiredLocation(pos);
    }
  }
}

// void USB_LP_CAN1_RX0_IRQHandler()
// {
//   if(CAN_GetITStatus(CAN1, CAN_IT_FMP0)){
//     CAN_Receive(CAN1, CAN_FIFO0,&rx_message);
//     if(rx_message.StdId == 18){
//         CAN_ReadPosition();
//         // timestamp = (uint8_t)0xFFFF & (CAN1->sFIFOMailBox->RDTR >> 16); 
//     }
//     CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//   }
// }




void USB_LP_CAN1_RX0_IRQHandler(void)
{
    if(CAN_GetITStatus(CAN1, CAN_IT_FMP0)){
      CanRxMsg rxMessage;
      while(CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
      {
          CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);
          can_rx_cnt++;
          CAN_InterpretMesssages(rxMessage);
      }
      CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
    if(CAN_GetITStatus(CAN1, CAN_IT_FOV0)){
        // There has been a buffer overflow
        can_overflow_cnt++;
        CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);
    }
    if(CAN_GetITStatus(CAN1, CAN_IT_FF0)){
        // Buffers are all full
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
    } 

}