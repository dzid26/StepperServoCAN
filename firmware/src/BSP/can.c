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
#include "nonvolatile.h"

const uint8_t n_base_msgs = 3U; // there are two currently but assume three for the future
static uint16_t can_command_id;
static uint16_t can_status_id;

static volatile uint32_t can_rx_cnt = 0U;      // cppcheck-suppress  misra-c2012-8.9
static volatile uint32_t can_tx_cnt = 0U;      // cppcheck-suppress  misra-c2012-8.9
volatile uint32_t can_err_rx_cnt = 0U;
static volatile uint32_t can_err_tx_cnt = 0U;  // cppcheck-suppress  misra-c2012-8.9
static volatile uint32_t can_overflow_cnt = 0U;// cppcheck-suppress  misra-c2012-8.9
static volatile uint8_t can_err_tx_last = 0U;  // cppcheck-suppress  misra-c2012-8.9

#define CAN_STID_SHIFT 5U
void CAN_MsgsFiltersSetup(void) {
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0U;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
	//IdList mode - fields below can store list of 4 receiving IDs.  STID requires << 5 
	CAN_FilterInitStructure.CAN_FilterIdHigh = can_command_id << CAN_STID_SHIFT; // default MSG_STEERING_COMMAND_FRAME_ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x700U << CAN_STID_SHIFT; //TODO add servicing message
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000U << CAN_STID_SHIFT;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000U << CAN_STID_SHIFT;
	//End CAN_FilterMode_IdList
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;

	CAN_FilterInit(&CAN_FilterInitStructure);

	//enable receive interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE); 
	CAN_ITConfig(CAN1, CAN_IT_FOV0, ENABLE); 
	}

void CAN_Setup(void) {
	can_command_id = nvmMirror.can.cmdId;
	can_status_id = nvmMirror.can.statusID;

	// shift can ids according to jumper configuration
	uint8_t can_config = Get_jumpers_config();
	can_command_id = can_command_id + n_base_msgs * can_config;
	can_status_id = can_status_id + n_base_msgs * can_config;

	CAN_MsgsFiltersSetup();
}

// Return checksum is lower byte of added lower and upper 
// bytes of 16bit sum of data values and message id
static uint8_t Msg_calc_checksum_8bit(const uint8_t *data, uint8_t len, uint16_t msg_id) {
	uint16_t checksum = msg_id;
	for(uint8_t i = 0U; i < len; i++){
		checksum += data[i];
	}
	checksum = (checksum & 0xFFU) + (checksum >> 8U); 
	checksum &= 0xFFU;

	return (uint8_t) checksum;
}

static void CheckTxStatus(uint8_t transmitMailbox) {
	uint8_t i = 0U;
	volatile uint8_t status = CAN_TxStatus_Failed;
	// wait until CAN transmission is OK
	while(status != CAN_TxStatus_Ok) {
		if(transmitMailbox == CAN_TxStatus_NoMailBox) {
			status = CAN_GetLastErrorCode(CAN1);
		}else{
			status = CAN_TransmitStatus(CAN1, transmitMailbox);
		}
		i++;
		if((status == CAN_TxStatus_Failed) || (i == UINT8_MAX)) {
			can_err_tx_cnt++;
			can_err_tx_last = CAN_GetLastErrorCode(CAN1);
			assert((can_err_tx_last == CAN_ErrorCode_NoErr) || (can_err_tx_last == CAN_ErrorCode_ACKErr)); //CAN_ErrorCode_ACKErr is allowed since it can happen if there is no CAN receivers on the bus
			return;
		}
	}
	can_tx_cnt++;
}


void CAN_TransmitMotorStatus(uint32_t frame) {
	CanTxMsg txMessage;
	txMessage.RTR=CAN_RTR_DATA;
	txMessage.IDE=CAN_ID_STD;
	txMessage.StdId=can_status_id; // default MSG_STEERING_STATUS_FRAME_ID
	txMessage.DLC=MSG_STEERING_STATUS_LENGTH;

	// populate message structure:
	struct Msg_steering_status_t controlStatus;
	controlStatus.checksum = 0U;
	controlStatus.counter = frame & 0xFU;
	controlStatus.steering_angle = Msg_steering_status_steering_angle_encode(StepperCtrl_getAngleFromEncoder());
	controlStatus.steering_speed = Msg_steering_status_steering_speed_encode(StepperCtrl_getSpeedRev());
	controlStatus.steering_torque = Msg_steering_status_steering_torque_encode(StepperCtrl_getControlOutput());
	controlStatus.temperature = Msg_steering_status_temperature_encode(GetChipTemp());
	uint16_t states = StepperCtrl_getStatuses();
	controlStatus.control_status = states & UINT8_MAX;
	controlStatus.debug_states = (states >> 8U)  & 0xFFU;
	
	// calculate checksum:
	uint8_t dataTemp[MSG_STEERING_STATUS_LENGTH];
	Msg_steering_status_pack(dataTemp, &controlStatus, sizeof(dataTemp));
	controlStatus.checksum = Msg_calc_checksum_8bit(dataTemp, MSG_STEERING_STATUS_LENGTH, can_status_id);
	Msg_steering_status_pack((&txMessage)->Data, &controlStatus, sizeof(txMessage.Data)); //pack again with the checksum

	// transmit
	uint8_t _transmitMailbox=CAN_Transmit(CAN1, &txMessage);
	CheckTxStatus(_transmitMailbox);
}

static volatile uint16_t can_control_cmd_cnt = 0U;
struct Msg_steering_command_t ControlCmds;
static void CAN_InterpretMesssages(CanRxMsg message) { 
	if(message.StdId == can_command_id) {
		Msg_steering_command_unpack(&ControlCmds, message.Data, sizeof(message.Data));
		// Note signals may correspond to different motor sample
		StepperCtrl_setControlMode(ControlCmds.steer_mode); //set control mode
		StepperCtrl_setFeedForwardTorque(Msg_steering_command_steer_torque_decode(ControlCmds.steer_torque));
		StepperCtrl_setDesiredAngle(Msg_steering_command_steer_angle_decode(ControlCmds.steer_angle));

		//calculate checksum:
		message.Data[0] = 0U; //!clear checksum - make sure which byte is checksum
		uint8_t checksum = Msg_calc_checksum_8bit(message.Data, MSG_STEERING_COMMAND_LENGTH, can_command_id);
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

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0)) {
		CanRxMsg rxMessage;
		while(CAN_MessagePending(CAN1, CAN_FIFO0) > 0) {
				CAN_Receive(CAN1, CAN_FIFO0, &rxMessage);
				can_rx_cnt++;
				CAN_InterpretMesssages(rxMessage);
		}
	}
	if(CAN_GetITStatus(CAN1, CAN_IT_FOV0)) {
			// There has been a buffer overflow
			can_overflow_cnt++;
			CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);
	}
	if(CAN_GetITStatus(CAN1, CAN_IT_FF0)) {
			// Buffers are all full
			CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
	}

}

#define CHECK_RX_FAIL_LIM 5U
// called from 10ms task
bool CAN_rx_validate_tick(void) {
	static uint16_t can_control_cmds_cnt_prev = 0U;
	static uint8_t rx_fail_cnt = 0U;


	if (can_control_cmd_cnt != can_control_cmds_cnt_prev) { //counter has changed - OK
		can_control_cmds_cnt_prev = can_control_cmd_cnt;
		rx_fail_cnt = 0U; //reset counter
		return true;
	}else{
		rx_fail_cnt++;
		if (rx_fail_cnt > CHECK_RX_FAIL_LIM) {
			rx_fail_cnt = CHECK_RX_FAIL_LIM; 
			return false;
		}
		return true;
	}
}