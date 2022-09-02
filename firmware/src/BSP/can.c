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


//todo: pass pointer to structure like rxMessage to limit stack size
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "control_api.h"
#include "Msg.h"

volatile uint32_t can_rx_cnt = 0;
volatile uint32_t can_tx_cnt = 0;
volatile uint32_t can_err_tx_cnt = 0;
volatile uint32_t can_err_rx_cnt = 0;
volatile uint32_t can_overflow_cnt = 0;

static int GetTemperature()
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);  
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)!=SET);
  volatile float ADCVolt = ADC_GetConversionValue(ADC1)*3.3f/4096;
  //todo add this calibration as a display feature
  const float t0 = 35.0f;
  const float ADCVoltRef = 1.325f; //! calibrate at some t0
  const float TempSlop = 4.3f/1000; //typically 4.3mV per C
  float fTemp = (ADCVoltRef - ADCVolt) / TempSlop + t0;
  return (int)(fTemp);

}


void CAN_MsgsFiltersSetup()
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  //IdList mode - fields below can store list of 4 receiving IDs.  STID requires << 5 
	CAN_FilterInitStructure.CAN_FilterIdHigh=MSG_STEERING_COMMAND_FRAME_ID<<5; 
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000<<5;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000 <<5;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000 <<5;  
  //End CAN_FilterMode_IdList
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
  }

// Return checksum is lower byte of added lower and upper 
// bytes of 16bit sum of data values and message id
uint8_t Msg_calc_checksum_8bit(const uint8_t *data, uint8_t len, uint16_t msg_id){
  uint16_t checksum = msg_id;
  for(uint8_t i = 0; i < len; i++){
    checksum += data[i];
  }
  checksum = (checksum & 0xFFu) + (checksum >> 8); 
  checksum &= 0xFFu;

  return (uint8_t) checksum;
}

struct Msg_steering_status_t ControlStatus;

void CAN_TransmitMotorStatus(uint32_t frame)
{
  //populate message structure:
  ControlStatus.counter = frame & 0xF;
  ControlStatus.steering_angle = Msg_steering_status_steering_angle_encode(StepperCtrl_getAngleFromEncoder());
  ControlStatus.steering_speed = Msg_steering_status_steering_speed_encode(StepperCtrl_getSpeedRev());
  ControlStatus.steering_torque = Msg_steering_status_steering_torque_encode(StepperCtrl_getControlOutput());
  ControlStatus.temperature = Msg_steering_status_temperature_encode(GetTemperature());
  uint16_t states = StepperCtrl_getStatuses();
  ControlStatus.control_status = states & 0xFF;
  ControlStatus.debug_states = (states >> 8)  & 0xFF;
  
  //calculate checksum:
  uint8_t DataTemp[8];
  Msg_steering_status_pack(DataTemp, &ControlStatus, sizeof(DataTemp));
  ControlStatus.checksum = Msg_calc_checksum_8bit(DataTemp, MSG_STEERING_STATUS_LENGTH, MSG_STEERING_STATUS_FRAME_ID);

  //pack and transmit
  CanTxMsg TxMessage;
  uint8_t TransmitMailbox;
  TxMessage.RTR=CAN_RTR_DATA;
  TxMessage.IDE=CAN_ID_STD;

  Msg_steering_status_pack((&TxMessage)->Data, &ControlStatus, sizeof(TxMessage.Data));
  TxMessage.StdId=MSG_STEERING_STATUS_FRAME_ID;
  TxMessage.DLC=MSG_STEERING_STATUS_LENGTH;
  TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);

  // TxMessage.StdId=0xC4; //todo remove debug
  // extern int16_t pTerm_glob;
  // extern int16_t iTerm_glob;
  // extern int16_t dTerm_glob;
  // TxMessage.DLC=8;
  // TxMessage.Data[0] = (StepperCtrl_getAngleFromEncoder() / RAW_POSITION_TO_MOTOR) & 0xFF;
  // TxMessage.Data[1] = (StepperCtrl_getAngleFromEncoder() / RAW_POSITION_TO_MOTOR) >> 8;
  // TxMessage.Data[3] =  StepperCtrl_getSpeedRev() & 0xFF;
  // TxMessage.Data[4] =  StepperCtrl_getSpeedRev() >> 8;
  
  // TxMessage.Data[5] = (pTerm_glob / RAW_TORQUE_TO_mA) & 0xFF;
  // TxMessage.Data[6] = (iTerm_glob / RAW_TORQUE_TO_mA) & 0xFF;
  // TxMessage.Data[7] = (dTerm_glob / RAW_TORQUE_TO_mA) & 0xFF;
  // TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);
  
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
      can_err_tx_cnt++;
      status = CAN_GetLastErrorCode(CAN1);
      return;
    }
  }
  can_tx_cnt++;
}


static volatile uint16_t can_control_cmd_cnt = 0;
struct Msg_steering_command_t ControlCmds;
void CAN_InterpretMesssages(CanRxMsg message) { 
  switch (message.StdId){
  	case MSG_STEERING_COMMAND_FRAME_ID: {      
      Msg_steering_command_unpack(&ControlCmds, message.Data, sizeof(message.Data));
      // Note signals may correspond to different motor sample
      StepperCtrl_setDesiredAngle(Msg_steering_command_steer_angle_decode(ControlCmds.steer_angle));
      StepperCtrl_setFeedForwardTorque(Msg_steering_command_steer_torque_decode(ControlCmds.steer_torque));
      StepperCtrl_setControlMode(ControlCmds.steer_mode); //set control mode
      
      //calculate checksum:
      message.Data[0] = 0; //!clear checksum - make sure which byte is checksum
      uint8_t checksum = Msg_calc_checksum_8bit(message.Data, MSG_STEERING_COMMAND_LENGTH, MSG_STEERING_COMMAND_FRAME_ID);
      #ifdef IGNORE_CAN_CHECKSUM
        ControlCmds.checksum = checksum;
      #endif
      if (ControlCmds.checksum == checksum){
        can_control_cmd_cnt++; //if this counter is not incremented, the error will be raised
      } else {
        can_err_rx_cnt++;
      }
      // todo also check counter is rolling by 1
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

bool Check_Control_CAN_rx_validate_tick(void) //call from 10ms task
{
  static uint16_t can_control_cmds_cnt_prev = 0;
  static uint8_t rx_fail_cnt = 0;


  if (can_control_cmd_cnt != can_control_cmds_cnt_prev) //counter has changed - OK
  {
    can_control_cmds_cnt_prev = can_control_cmd_cnt;
    rx_fail_cnt = 0; //reset counter
    return true;
  }else{
    rx_fail_cnt++;
    if (rx_fail_cnt > CHECK_RX_FAIL_LIM)
    {
      rx_fail_cnt = CHECK_RX_FAIL_LIM; 
      return false;
    }
    return true;
  }
}