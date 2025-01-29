//! This file is not used

 /**
 * StepperServoCAN
 *
 * Copyright (c) 2020 Makerbase.
 *
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

#include "MKS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "actuator_config.h"
#include "control_api.h"
#include "can.h"
#include "board.h"
#include "calibration.h"
#include "nonvolatile.h"
#include "display.h"
#include "encoder.h"

static void displayError(uint16_t error){
	int32_t x,y;
	char str[25];
	x = (36000 * (int32_t)error) / ANGLE_STEPS;
	y = x / 100;
	x = x - (y * 100);
	x = (x < 0) ? 0 : x;
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
	error = EncoderCalibrate(false);
	displayError(error);
	return 1;
}

int menuTestCal(int argc, char *argv[])
{
	uint16_t error;
	display_show("", "Testing Cal...", "", "");
	error = EncoderCalibrate(true);
	displayError(error);
	return 1;
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
		i = atol(argv[0]);
		if (i != nvmMirror.systemParams.controllerMode)
		{
			nvmMirror.systemParams.controllerMode = (feedbackCtrl_t)i;
			nvmWriteConfParms();
		}
		return i;
	}
	return nvmMirror.systemParams.controllerMode;
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
		i = atol(argv[0]);
		if (i != nvmMirror.systemParams.errorPinMode)
		{
			nvmMirror.systemParams.errorPinMode = (ErrorPinMode_t)i;
			nvmWriteConfParms();
		}
		return i;
	}
	return nvmMirror.systemParams.errorPinMode;
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
		i = atol(argv[0]);
		if (i != nvmMirror.systemParams.dirRotation)
		{
			nvmMirror.systemParams.dirRotation = (RotationDir_t)i;
			nvmWriteConfParms();
		}
		return i;
	}
	return nvmMirror.systemParams.dirRotation;
}

menuItem_t MenuMain[] = {
		{"Calibrate", menuCalibrate, NULL},
		{"Test Cal", menuTestCal, NULL},
		{"EnablePin", enablePin, enablePinOptions},
		{"Rotation", changeDir, dirOptions},
		{ "", NULL, NULL}
};

menuItem_t MenuCal[] = {
		{"Calibrate", menuCalibrate, NULL},
		{ "", NULL, NULL}
};