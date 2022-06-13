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
#include "control_api.h"
#include "../OP/Msg.h"

#define RAW_TORQUE_TO_mA 1
#define RAW_TORQUE_MAX_TO_mA 1
#define RAW_POSITION_TO_MOTOR 64

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
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  //IdList mode - fields below can store list of 4 IDs.  STID requires << 5 
	CAN_FilterInitStructure.CAN_FilterIdHigh=MSG_CONTROL_CMD1_FRAME_ID<<5; 
	CAN_FilterInitStructure.CAN_FilterIdLow=MSG_LIMITS_CMD2_FRAME_ID<<5;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000 <<5;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000 <<5;  
  //End CAN_FilterMode_IdList
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
  }

struct Msg_control_status1_t ControlStatus;
struct Msg_system_status2_t SystemStatus;
void CAN_TransmitMotorStatus(void)
{
  static uint8_t counter = 0;
  ControlStatus.counter_stat1 = (counter++) & 0xF;
  ControlStatus.torque_actual = StepperCtrl_getControlOutput() / RAW_TORQUE_TO_mA;
  ControlStatus.torque_close_loop_actual = StepperCtrl_getCloseLoop() / RAW_TORQUE_TO_mA;
  ControlStatus.position_error = StepperCtrl_getPositionError() / RAW_POSITION_TO_MOTOR;
  ControlStatus.speed_actual =  StepperCtrl_getSpeedRev();

  // SystemStatus.chip_temp = GetTemperature();
  SystemStatus.position_raw = StepperCtrl_getCurrentLocation();
  
  /* transmit */
  CanTxMsg TxMessage;
  uint8_t TransmitMailbox;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.IDE=CAN_ID_STD;

  Msg_control_status1_pack((&TxMessage)->Data, &ControlStatus, sizeof(TxMessage.Data));
  TxMessage.StdId=MSG_CONTROL_STATUS1_FRAME_ID;
  TxMessage.DLC=MSG_CONTROL_STATUS1_LENGTH;
  TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);

  Msg_system_status2_pack((&TxMessage)->Data, &SystemStatus, sizeof(TxMessage.Data));
  TxMessage.StdId=MSG_SYSTEM_STATUS2_FRAME_ID;
  TxMessage.DLC=MSG_SYSTEM_STATUS2_LENGTH;
  TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);
  
  CheckTxStatus(TransmitMailbox);

}

void CheckTxStatus(uint8_t TransmitMailbox){
	uint8_t i = 0;
	uint8_t status = CAN_TxStatus_Failed;
  // wait until CAN transmission is OK
  i = 0;
  while((status != CANTXOK))               
  {
    status = CAN_TransmitStatus(CAN1, TransmitMailbox);
    i++;
    if (status == CAN_TxStatus_Failed || (i == 0xFF)){
      can_err_cnt++;
      status = CAN_GetLastErrorCode(CAN1);
      return;
    }
  }
  can_tx_cnt++;
}


struct Msg_control_cmd1_t ControlCmds;
void CAN_InterpretMesssages(CanRxMsg message) { 
  switch (message.StdId){
  	case MSG_CONTROL_CMD1_FRAME_ID: {
      Msg_control_cmd1_unpack(&ControlCmds, message.Data, sizeof(message.Data));

      StepperCtrl_setDesiredLocation( (int32_t) ControlCmds.position_change * RAW_POSITION_TO_MOTOR);
      StepperCtrl_setFeedForwardTorque(ControlCmds.torque_feedforward * RAW_TORQUE_TO_mA); //set feedforward torque
      StepperCtrl_setCloseLoopTorque(ControlCmds.torque_closeloop_max * RAW_TORQUE_MAX_TO_mA); //set feedforward torque
      StepperCtrl_setControlMode(ControlCmds.target_mode); //set control mode
      
    }
  }
}

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