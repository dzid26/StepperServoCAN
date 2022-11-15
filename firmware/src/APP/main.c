 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
 * Copyright (C) 2018 MisfitTech LLC.
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

#include "main.h"
#include "MKS.h"
#include "stepper_controller.h"
#include "encoder.h"
#include "board.h"
#include "can.h"
#include "control_api.h"
#include "calibration.h"
#include "nonvolatile.h"
#include "actuator_config.h"
#include "display.h"
#include "delay.h"

extern void initialise_monitor_handles(void); //semihosting


static volatile bool runCalibration = false;
static void RunCalibration(void){
	bool state = motion_task_isr_enabled;
	Motion_task_disable();
	Set_Error_LED(true);

	bool err1 = false;
	bool err2 = false;

	do{
		//print errors on after failed calibration
		if(err1){
			(void) printf("Calibration not set\n");
		}else if (err2){
			(void) printf("Large deviation. Reposition the magnet\n");
		}
		
		(void) printf("Confirm new calibration start..\n");
		do{	//wait for the user
			Set_Func_LED(true);
		}while(!Fcn_button_state() && getchar());
		Set_Func_LED(false);

		//print angle using fixed point
		uint16_t max_error100 = (uint16_t) (ANGLERAW_T0_DEGREES(StepperCtrl_calibrateEncoder(true))*100.0f);
		uint16_t max_error = max_error100/100u;
		(void) printf("Max deviation was %01u.%02u deg\n", max_error, (max_error*100u)-max_error100);

		//assert errors
		err1 = !CalibrationTable_calValid();
		err2 = max_error > DEGREES_TO_ANGLERAW(1);
	}while(err1 || err2);

	(void) printf("Calibration ok\n");

	if(state){
		Motion_task_enable();
	}
}


volatile stepCtrlError_t stepCtrlError = STEPCTRL_NO_POWER;
static void Begin_process(void)
{
	
	update_actuator_parameters();

	board_init();	//set up the pins correctly on the board.

	nonvolatile_begin(); //����NVM_address

	validateAndInitNVMParams(); //systemParams init

	display_begin(); //display init
	Serivice_task_init(); //task init

	display_show("StepperServoCAN", "initialization..", "", "");
	delay_ms(1);
	Set_Error_LED(true);
	stepCtrlError = STEPCTRL_NO_CAL;
	while(STEPCTRL_NO_ERROR != stepCtrlError)
	{
		//start controller before accepting step inputs
		stepCtrlError = StepperCtrl_begin();

		//start up encoder
		if (STEPCTRL_NO_ENCODER == stepCtrlError)
		{
			display_show("Encoder", " Error!", "REBOOT", "");
			while(1);	//����
		}

		if(STEPCTRL_NO_POWER == stepCtrlError)
		{
			display_show("Waiting", "MOTOR", "POWER", "");
			while(STEPCTRL_NO_POWER == stepCtrlError)
			{
				stepCtrlError = StepperCtrl_begin(); //start controller before accepting step inputs
			}
		}

		if(STEPCTRL_NO_CAL == stepCtrlError)
		{
			display_show("   NOT ", "Calibrated", " ", "");
			delay_ms(2200);
			display_process();
			RunCalibration();
		}
	}
	display_setMenu(MenuMain);
	printf("Initialization successful\n");
	Set_Error_LED(false);

	printf("Starting motion task\n");
	StepperCtrl_enable(true);
	
}


static void Background_process(void){
	if(runCalibration){
		RunCalibration();
		runCalibration = false;
	}
}

//fast motor control task
//rolling counters for debugging
volatile uint32_t motion_task_counter=0;	// cppcheck-suppress  misra-c2012-8.4
volatile uint32_t motion_task_count=0;		// cppcheck-suppress  misra-c2012-8.4
void Motion_task(void){
	motion_task_count++;

	(void) StepperCtrl_processMotion(); //handle the control loop
}

//10ms task for communication and diagnostic
void Service_task(void){
	motion_task_counter++;

	ChipTemp_adc_update();
	Vmot_adc_update();

	//transmit CAN every 10ms
	CAN_TransmitMotorStatus(motion_task_counter);

	//go to Soft Off if motor is actively controlled but control signal is not received
	bool comm_error = false;
	if(enableFeedback){
		comm_error = (Check_Control_CAN_rx_validate_tick() == false);
	}
	if(comm_error)
	{	//once SOFT_TORQUE_OFF is set, the motor will not be controlled until STEPCTRL_OFF is requested
		StepperCtrl_setControlMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}

	//Function button and LED processing
	static uint16_t fcn_button_count = 0; //centiseconds
	const uint16_t fcn1_button_delay = 300U;//hold 3s to trigger re-calibration
	if(Fcn_button_state()){
		fcn_button_count++;
		StepperCtrl_setControlMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
	if(fcn_button_count == (fcn1_button_delay-10U))	{Set_Func_LED(true);} 	//short
	if(	fcn_button_count == fcn1_button_delay)		{Set_Func_LED(false);}	//blink
	if((fcn_button_count >= fcn1_button_delay)  && (!Fcn_button_state())){ 	//wait for button release
		runCalibration = true; 		//request calibration - it will run in Background_process()
	}
	if(!Fcn_button_state()){
		fcn_button_count=0;
	}
}

int main (void)
{	
	#ifdef DEBUG
	initialise_monitor_handles();
	#endif
	
	Begin_process();
	while(1)
	{
		Background_process();
	}
}