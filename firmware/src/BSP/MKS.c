 /**
 * MKS SERVO42B
 * Copyright (c) 2020 Makerbase.
 *
 * Based on nano_stepper project by Misfittech
 * Copyright (C) 2018  MisfitTech LLC.
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

#include "MKS.h"

extern volatile bool StepperCtrl_Enabled;
extern volatile uint32_t NVM_address;

nvm_t nvmParams = {0};

static void displayError(uint16_t error){
	int32_t x,y;
	char str[25];
	x = (36000 * (int32_t)error) / ANGLE_STEPS;
	y = x / 100;
	x = x - (y * 100);
	x = fastAbs(x);
	sprintf(str, "%ld.%02ld deg",y,x);
	display_show("Max cal error", str, " -> Press Enter", "to continue");

	while(GPIO_ReadInputDataBit(PIN_SW, PIN_SW3_ENTER) == 1)
	{
		//wait for button press
	}
	while(GPIO_ReadInputDataBit(PIN_SW, PIN_SW3_ENTER) == 0)
	{
		//wait for button release
	}
}

int menuCalibrate(int argc, char *argv[])
{
	uint16_t error;
	display_show("", "Calibrating...", "", "");
	error = StepperCtrl_calibrateEncoder(true);
	displayError(error);
	return 1;
}

int menuTestCal(int argc, char *argv[])
{
	uint16_t error;
	display_show("", "Testing Cal...", "", "");
	error = StepperCtrl_calibrateEncoder(false);
	displayError(error);
	return 1;
}

options_t stepOptions[] = {
		{"200"},
		{"400"},
		{""},
};

//returns the index of the stepOptions when called
// with no arguments.
int motorSteps(int argc, char *argv[])
{
	if (argc == 0)
	{
		uint16_t i;
		i = NVM->motorParams.fullStepsPerRotation;
		if (i == 400)
		{
			return 1;
		}
		return 0; //default to 200
	}
	if (argc > 0)
	{
		uint16_t i;
		nvm_t params;
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		i = atol(argv[0]);
		if (i != params.motorParams.fullStepsPerRotation)
		{
			params.motorParams.fullStepsPerRotation = i;
			nvmWriteConfParms(&params);
		}
	}
	return 0;
}

options_t currentOptions[] = {
		{"0"},
		{"100"},
		{"200"},
		{"300"},
		{"400"},
		{"500"},
		{"600"},
		{"700"},
		{"800"},
		{"900"},
		{"1000"},
		{"1100"},
		{"1200"},
		{"1300"},
		{"1400"},
		{"1500"},
		{"1600"},
		{"1700"},
		{"1800"},
		{"1900"},
		{"2000"},
#ifndef MKS_SERVO42B
		{"2100"},
		{"2200"},
		{"2300"},
		{"2400"},
		{"2500"},
		{"2600"},
		{"2700"},
		{"2800"},
		{"2900"},
		{"3000"},
		{"3100"},
		{"3200"},
		{"3300"},
#endif
		{""},
};


options_t currentHoldOptions[] = {
		{"0"},
		{"100"},
		{"200"},
		{"300"},
		{"400"},
		{"500"},
		{"600"},
		{"700"},
		{"800"},
		{"900"},
		{"1000"},
		{"1100"},
		{"1200"},
		{"1300"},
		{"1400"},
		{"1500"},
#ifndef MKS_SERVO42B
		{"1600"},
		{"1700"},
		{"1800"},
		{"1900"},
		{"2000"},
#endif
		{""},
};

int motorCurrent(int argc, char *argv[])
{
	if (argc == 1)
	{
		int i;
		nvm_t params;
		i = atol(argv[0]);
		i = i * 100;
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		if (i != params.motorParams.currentMa)
		{
			if(i > 3300)
			{
				i = 3300;
			}
			params.motorParams.currentMa = i;
			nvmWriteConfParms(&params);
		}
		return i / 100;
	}else
	{
		int i;
		i = NVM->motorParams.currentMa / 100;
		return i;
	}
}

int motorHoldCurrent(int argc, char *argv[])
{
	if (argc == 1)
	{
		uint16_t i;
		nvm_t params;
		i = atol(argv[0]);
		i = i * 100;
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		if (i != params.motorParams.currentHoldMa)
		{
			if(i > 3300)
			{
				i = 3300;
			}
			params.motorParams.currentHoldMa = i;
			nvmWriteConfParms(&params);
		}
		return i / 100;

	}else
	{
		uint16_t i;
		i = NVM->motorParams.currentHoldMa / 100;
		return i;
	}
}

options_t microstepOptions[] = {
		{"1"},
		{"2"},
		{"4"},
		{"8"},
		{"16"},
		{"32"},
		{"64"},
		{"128"},
		{"256"},
		{""}
};

int microsteps(int argc, char *argv[])
{
	if (argc == 1)
	{
		uint16_t i,steps;
		nvm_t params;
		i = atol(argv[0]);
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		steps = 0x01 << i;
		if (steps != params.SystemParams.microsteps)
		{
			params.SystemParams.microsteps = steps;
			nvmWriteConfParms(&params);
		}
		return i;
	}else
	{
		uint16_t i,j;
		i = NVM->SystemParams.microsteps;
		for (j=0; j<9; j++)
		{

			if ((0x01<<j) == i)
			{
				return j;
			}
		}
		return 0;
	}
}

options_t controlLoopOptions[] = {
		{"Simple"},
		{"Pos PID"},
		{"Vel PID"},
		{""}
};

int controlLoop(int argc, char *argv[])
{
	if (argc == 1)
	{
		uint16_t i;
		nvm_t params;
		i = atol(argv[0]);
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		if (i != params.SystemParams.controllerMode)
		{
			params.SystemParams.controllerMode = (feedbackCtrl_t)i;
			nvmWriteConfParms(&params);
		}
		return i;
	}
	return NVM->SystemParams.controllerMode;
}

options_t enablePinOptions[] = {
		{"Enable"},
		{"!Enable"},
		{""}
};

int enablePin(int argc, char *argv[])
{
	if (argc == 1)
	{
		uint16_t i;
		nvm_t params;
		i = atol(argv[0]);
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		if (i != params.SystemParams.errorPinMode)
		{
			params.SystemParams.errorPinMode = (ErrorPinMode_t)i;
			nvmWriteConfParms(&params);
		}
		return i;
	}
	return NVM->SystemParams.errorPinMode;
}

options_t dirOptions[] = {
		{"High CW"},
		{"High CCW"},
		{""}
};

int changeDir(int argc, char *argv[])
{
	if (argc == 1)
	{
		uint16_t i;
		nvm_t params;
		i = atol(argv[0]);
		memcpy((void *)&params, (void *)NVM, sizeof(params));
		if (i != params.SystemParams.dirRotation)
		{
			params.SystemParams.dirRotation = (RotationDir_t)i;
			nvmWriteConfParms(&params);
		}
		return i;
	}
	return NVM->SystemParams.dirRotation;
}

menuItem_t MenuMain[] = {	//���˵�
		{"Calibrate", menuCalibrate, NULL},
		{"Test Cal", menuTestCal, NULL},
		{"Max mA", motorCurrent, currentOptions},
		{"Hold mA", motorHoldCurrent, currentHoldOptions},
		// {"Current mA", motorCurrent, currentOptions},
		{"Microstep", microsteps, microstepOptions},
		{"EnablePin", enablePin, enablePinOptions},
		{"Rotation", changeDir, dirOptions},
		{ "", NULL, NULL}
};

menuItem_t MenuCal[] = {	//Calibrate�˵�
		{"Calibrate", menuCalibrate, NULL},
		{ "", NULL, NULL}
};

//check the NVM and set to defaults if there is any
void validateAndInitNVMParams(void)
{
	if (NVM->SystemParams.parametersValid != valid) //SystemParams invalid
	{
		nvmParams.sPID.Kp = .1;  nvmParams.sPID.Ki = .008;  nvmParams.sPID.Kd = 0.2;
		nvmParams.pPID.Kp = 1.0;  nvmParams.pPID.Ki = 0.0; 	  nvmParams.pPID.Kd = 0.0;
		nvmParams.vPID.Kp = 2.0;  nvmParams.vPID.Ki = 1.0; 	  nvmParams.vPID.Kd = 1.0;

		nvmParams.SystemParams.microsteps = 256; //unused
		nvmParams.SystemParams.controllerMode = CTRL_SIMPLE;
		nvmParams.SystemParams.dirRotation = CCW_ROTATION;
		nvmParams.SystemParams.errorLimit = (int32_t)ANGLE_FROM_DEGREES(1.8);
		nvmParams.SystemParams.errorPinMode = ERROR_PIN_MODE_ACTIVE_LOW_ENABLE;  //default to !enable pin

		if(NVM->motorParams.parametersValid == valid)
		{
			nvmParams.motorParams = NVM->motorParams;
			nvmWriteConfParms(&nvmParams);
		}
	}
	//the motor parameters are check in the stepper_controller code
	// as that there we can auto set much of them.
}

void Begin_process(void)
{
	stepCtrlError_t stepCtrlError;

	board_init();	//set up the pins correctly on the board.

	nonvolatile_begin(); //����NVM_address

	validateAndInitNVMParams(); //systemParams init

	oled_begin();
	display_begin(); //display init
	Task_10ms_init(); //task init

  	#ifdef MKS_SERVO42B
  	display_show("MKS", "Servo42B", VERSON, ""); //��ʾ57LOGO
  	#elif S42Bv2
	  display_show("BTT", "S42Bv2", VERSON, ""); //��ʾ42LOGO
  	#else
	  display_show("MKS", "Servo57B", VERSON, ""); //��ʾ42LOGO
  	#endif
	delay_ms(800);
	
	WORK_LED(true);
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
			display_setMenu(MenuCal);
			display_forceMenuActive();

			while(CalibrationTable_calValid() != true)
			{
				display_process();
			}
		}
	}
	display_setMenu(MenuMain);
	StepperCtrl_enable(true);

	WORK_LED(false);
}


volatile uint32_t Task_10ms_counter;
volatile bool Task_10ms_overrun;
volatile uint32_t Task_10ms_overrun_count;
volatile uint16_t Task_10ms_execution_us;

volatile uint32_t Task_Motor_count;
volatile bool Task_Motor_overrun;
volatile uint32_t Task_Motor_overrun_count;
volatile uint16_t Task_Motor_execution_us;

volatile uint32_t Background_counter;

void Background_process(void){
	Background_counter++;

	display_process();
}

void Task_motor(void){
	Task_Motor_count++;

	bool no_error = false;
	no_error = StepperCtrl_processFeedback(); //handle the control loop
	// WORK_LED(no_error);
}

void Task_10ms(void){
	Task_10ms_counter++;

	CAN_TransmitMotorStatus();
}