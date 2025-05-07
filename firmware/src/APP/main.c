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
 * along with this program.  If not, see <www.gnu.org/licenses/>.
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

#ifdef DEBUG
#include <stdio.h>

extern void initialise_monitor_handles(void); //semihosting

//similar to assert() but continues after pause
void debug_assert_func(const char *file, int line, const char *func, const char *failedexpr) {
	(void) printf("\nAssertion \"%s\" failed: file \"%s\", line %d%s%s\n",
		failedexpr, file, line,
		func ? ", function: " : "", func ? func : "");
		__ASM ("BKPT #02"); //debug on error and return, instead of newlib's abort
}
static void clean_stdin(void){
    int c;
    do {
        c = getchar();
    } while ((c != (int)'\n') && (c != EOF));
}

static char HostReadCharacter(void){ //semihosting read
    char cha=0;
    scanf("%c", &cha);
    // Discard extra characters in input buffer
    clean_stdin(); //discard characters after the number
    return cha;
}
#else
static char HostReadCharacter(void) { return 0; }
#endif //DEBUG

//STMicro assert called from stm32f10x_conf.h
void assert_failed(uint8_t* file, uint32_t line){
	__assert_func((char*) file, (int)line, NULL, "STM32 registers");
}

// confirm using a button
// if in debug mode, use the semihosting debug console
void User_confirmation(void){
	(void) printf("Press any ENTER key to proceed... (Press physical RESET button to cancel)\n");
	// delay_ms(2000); //make sure the printf above reaches the host
	int key;
	do{	//wait for the user
		Set_Func_LED(true);
		key = HostReadCharacter(); //press any key
		// printf("  key = %c\n", key);
	}while((!F1_button_state()) && (key == 0));
	Set_Func_LED(false);
}

static bool runCalibration = false;
static bool runKbemfEstimation = false;
static void RunCalibration(void){
	StepperCtrl_enable(false);
	apiAllowControl(false);
	runCalibration = true; //set again depending who calls the function

	Set_Error_LED(true);
	(void) printf("Calibration routines will be performed:\n");
	bool err0 = false;
	do{
		(void) printf("1. Motor type and wiring orientation detection\n");
		(void) printf("2. Magnet offset calibration.\n");
		User_confirmation();
		err0 = !Learn_StepSize_WiringPolarity();
		if (err0){
			(void) printf("ERROR: Motor blocked or unpowered. Retrying...\n");
		}else{
			(void) printf("Phase orientation: %d\n", (uint8_t)(liveMotorParams.invertedPhase));
			(void) printf("Motor steps: %d\n", liveMotorParams.fullStepsPerRotation);
		}
	}while(err0);

	bool err1 = false;
	bool err2 = false;

	do{//assert errors
		if (err1 || err2){
			(void) printf("Retrying encoder calibration.\n");
			User_confirmation(); //confirm retry
		}
		uint16_t max_error = EncoderCalibrate(false);
		float max_error_deg = ANGLERAW_T0_DEGREES(max_error);
		//print angle using fixed point
		(void) printf("Max deviation was %01u.%02u deg\n", (uint16_t)max_error_deg, (uint16_t)((uint32_t)(max_error_deg*100.0)%100U));

		//assert errors
		err1 = !CalibrationTable_calValid();
		err2 = max_error >= CALIBRATION_MAX_ERROR;
		//print errors on after failed calibration
		if(err1){
			(void) printf("Calibration not set\n");
		}
		if (err2){
			(void) printf("ERROR: Large deviation. Reposition the magnet\n");
			delay_ms(1000);
		}
	}while(err1 || err2);
	Set_Error_LED(false);
	(void) printf("Calibration OK\n");

	runCalibration = false;
	StepperCtrl_enable(true);
	apiAllowControl(true);
}


volatile stepCtrlError_t stepCtrlError = STEPCTRL_NO_POWER;
static void Begin_process(void) {
	board_init();	//set up the pins correctly on the board.

	update_actuator_parameters(USE_SIMPLE_PARAMETERS);

	nonvolatile_begin();
	validateAndInitNVMParams(); //systemParams init

	CAN_Setup(); // can id filters

	display_begin(); //display init
	Serivice_task_init(); //task init
	Motion_task_init(SAMPLING_PERIOD_uS);

	display_show("StepperServoCAN", "initialization..", "", "");
	Set_Error_LED(true);
	delay_ms(10); // wait for power to stabilize
	stepCtrlError = StepperCtrl_begin();
	while(stepCtrlError != STEPCTRL_NO_ERROR) {
		if(stepCtrlError == STEPCTRL_NO_POWER) {
			display_show("MOTOR", "POWER", "Waiting", "");
			Set_Error_LED(false);
			delay_ms(10); // short red led blink
		}else if(stepCtrlError == STEPCTRL_NO_CAL) {
			display_show("Encoder", " Not ", "calibrated", " ");
			delay_ms(2200);
			display_process();
			RunCalibration();
		}else if (stepCtrlError == STEPCTRL_NO_ENCODER) {
			display_show("Encoder", "Init Error!", "REBOOT", "");
			Set_Error_LED(false);
			delay_ms(1000); // long red led blink
		}else {
			assert(0);
		}
		// Retry after ~1s:
		Set_Error_LED(true);
		delay_ms(1500);
		stepCtrlError = StepperCtrl_begin();
	}
	Set_Error_LED(false);

	(void) printf("Initialization successful\n");

	(void) printf("Starting motion task\n");
	StepperCtrl_enable(true);

	apiAllowControl(true);
}


static void Background_process(void) {
	if(runCalibration){
		RunCalibration();
	}
	if(runKbemfEstimation){
		apiAllowControl(false);
		Estimate_motor_k_bemf();
		runKbemfEstimation = false;
		apiAllowControl(true);
	}
}

//fast motor control task
//rolling counters for debugging
volatile uint32_t motion_task_counter=0;	// cppcheck-suppress  misra-c2012-8.4
volatile uint32_t service_task_counter=0;	// cppcheck-suppress  misra-c2012-8.4
void Motion_task(void){
	motion_task_counter++;

	(void) StepperCtrl_processMotion(); //handle the control loop
}

//10ms task for communication and diagnostic
void Service_task(void) {
	service_task_counter++;

	adc_update_all();
	
	//transmit CAN every 10ms
	CAN_TransmitMotorStatus(service_task_counter);

	//go to Soft Off if motor is actively controlled but control signal is not received
	bool comm_error = false;
	if(enableSensored){
		comm_error = (Check_Control_CAN_rx_validate_tick() == false);
	}
	if(comm_error)
	{	//once SOFT_TORQUE_OFF is set, the motor will not be controlled until STEPCTRL_OFF is requested
		StepperCtrl_setControlMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}

	const uint16_t button_delay_calib = 200U;//hold 2s to trigger calibration
	//Function button and LED processing
	static uint16_t f1_button_count = 0; //centiseconds
	if(F1_button_state() && (stepCtrlError == STEPCTRL_NO_ERROR)){//look for button long press
		f1_button_count++;
		StepperCtrl_setControlMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
	if(f1_button_count == (button_delay_calib-10U))	{Set_Func_LED(true);} 	//short LED blink
	if(	f1_button_count == button_delay_calib)		{Set_Func_LED(false);}
	if((f1_button_count >= button_delay_calib)  && (!F1_button_state())){ 	//wait for button release
		runCalibration = true; 		//request calibration - it will run in Background_process()
	}
	if(!F1_button_state()){
		f1_button_count=0;
	}

	//Function button and LED processing
	static uint16_t f2_button_count = 0; //centiseconds
	if(F2_button_state() && (stepCtrlError == STEPCTRL_NO_ERROR)){//look for button long press
		f2_button_count++;
		StepperCtrl_setControlMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
	if(f2_button_count == (button_delay_calib-10U))	{Set_Func_LED(true);} 	//short LED blink
	if(	f2_button_count == button_delay_calib)		{Set_Func_LED(false);}
	if((f2_button_count >= button_delay_calib)  && (!F2_button_state())){ 	//wait for button release
		runKbemfEstimation = true;
	}
	if(!F2_button_state()){
		f2_button_count=0;
	}
}

#ifndef PIO_UNIT_TESTING 
int main (void) {
	#ifdef DEBUG
	initialise_monitor_handles();
	#endif
	
	Begin_process();
	while(1)
	{
		Background_process();
	}
}
#endif //PIO_UNIT_TESTING