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
 * along with this program.  If not, see <www.gnu.org/licenses/>.
 *
 */


//todo: pass pointer to structure like rxMessage to limit stack size
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "control_api.h"
#include "Msg.h"
#include "board.h"

static volatile uint32_t can_rx_cnt = 0;      // cppcheck-suppress  misra-c2012-8.9
       volatile uint32_t can_err_rx_cnt = 0;
static volatile uint32_t can_tx_cnt = 0;      // cppcheck-suppress  misra-c2012-8.9
static volatile uint32_t can_err_tx_cnt = 0;  // cppcheck-suppress  misra-c2012-8.9
static volatile uint32_t can_overflow_cnt = 0;// cppcheck-suppress  misra-c2012-8.9

void CAN_MsgsFiltersSetup()
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;
  //IdList mode - fields below can store list of 4 receiving IDs.  STID requires << 5 
	CAN_FilterInitStructure.CAN_FilterIdHigh=MSG_STEERING_COMMAND_FRAME_ID<<5; 
	CAN_FilterInitStructure.CAN_FilterIdLow=0x700<<5; //debuging message
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000 <<5;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000 <<5;  
  //End CAN_FilterMode_IdList
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	CAN_FilterInit(&CAN_FilterInitStructure);
  }

// Return checksum is lower byte of added lower and upper 
// bytes of 16bit sum of data values and message id
static uint8_t Msg_calc_checksum_8bit(const uint8_t *data, uint8_t len, uint16_t msg_id){
  uint16_t checksum = msg_id;
  for(uint8_t i = 0; i < len; i++){
    checksum += data[i];
  }
  checksum = (checksum & 0xFFu) + (checksum >> 8); 
  checksum &= 0xFFu;

  return (uint8_t) checksum;
}

static void CheckTxStatus(uint8_t transmitMailbox){
	uint8_t i = 0;
	uint8_t status = CAN_TxStatus_Failed;
  // wait until CAN transmission is OK
  i = 0;
  while((status != CANTXOK))               
  {
    status = CAN_TransmitStatus(CAN1, transmitMailbox);
    i++;
    if ((status == CAN_TxStatus_Failed) || (i == 0xFFU)){
      can_err_tx_cnt++;
      status = CAN_GetLastErrorCode(CAN1);
      return;
    }
  }
  can_tx_cnt++;
}


void CAN_TransmitMotorStatus(uint32_t frame){
  static struct Msg_steering_status_t controlStatus;
  //populate message structure:
  controlStatus.counter = frame & 0xFU;
  controlStatus.steering_angle = Msg_steering_status_steering_angle_encode(StepperCtrl_getAngleFromEncoder());
  controlStatus.steering_speed = Msg_steering_status_steering_speed_encode(StepperCtrl_getSpeedRev());
  controlStatus.steering_torque = Msg_steering_status_steering_torque_encode(StepperCtrl_getControlOutput());
  controlStatus.temperature = Msg_steering_status_temperature_encode(GetChipTemp());
  uint16_t states = StepperCtrl_getStatuses();
  controlStatus.control_status = states & 0xFFU;
  controlStatus.debug_states = (states >> 8U)  & 0xFFU;
  
  //calculate checksum:
  uint8_t dataTemp[8];
  Msg_steering_status_pack(dataTemp, &controlStatus, sizeof(dataTemp));
  controlStatus.checksum = Msg_calc_checksum_8bit(dataTemp, MSG_STEERING_STATUS_LENGTH, MSG_STEERING_STATUS_FRAME_ID);

  //pack and transmit
  CanTxMsg txMessage;
  uint8_t _transmitMailbox;
  txMessage.RTR=CAN_RTR_DATA;
  txMessage.IDE=CAN_ID_STD;

  Msg_steering_status_pack((&txMessage)->Data, &controlStatus, sizeof(txMessage.Data));
  txMessage.StdId=MSG_STEERING_STATUS_FRAME_ID;
  txMessage.DLC=MSG_STEERING_STATUS_LENGTH;
  _transmitMailbox=CAN_Transmit(CAN1, &txMessage);
  
  CheckTxStatus(_transmitMailbox);

}

static volatile uint16_t can_control_cmd_cnt = 0;
struct Msg_steering_command_t ControlCmds;
static void CAN_InterpretMesssages(CanRxMsg message) { 
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

#define CHECK_RX_FAIL_LIM 5
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