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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "display.h"

extern volatile SystemParams_t systemParams;
extern volatile int32_t zeroAngleOffset;
extern volatile int32_t loopError;
extern volatile int32_t speed_slow;
extern volatile int32_t desiredLocation;

menuItem_t*		ptrMenu;				//Menu Bar
uint8_t 			menuIndex;			//Menu bar index
options_t*		ptrOptions;			//Option bar
uint8_t 			optionIndex;		//Option bar index

uint8_t 			buttonState;		//Button status
bool 					menuActive;			//Activate the menu

void display_begin(void)
{
	buttonState = 0;
	menuActive = false;

	ptrMenu = NULL;
	menuIndex = 0;
	ptrOptions = NULL;
	optionIndex = 0;
}

//4 line
void display_show(char* line1, char* line2, char* line3, char* line4)
{
	puts(line1);
	puts(line2);
    puts(line3);
	puts(line4);
}

//set Menu Bar (MenuMain/MenuCal)
void display_setMenu(menuItem_t *pMenu)
{
	ptrMenu = pMenu;
	menuIndex = 0;
}

void display_showMenu(void)
{
	uint8_t i,j;
	char str[4][16] = {0};

	i = menuIndex;
	j = 0;
	while(ptrMenu[i].func!=NULL && j < 4)
	{
		if (i == menuIndex)
		{
			sprintf(str[j],"-> %s",ptrMenu[i].str);
		}else
		{
			sprintf(str[j],"   %s",ptrMenu[i].str);
		}
		j++;
		i++;
	}

	//show exit if there is room
	if (j < 4)
	{
		if (j == 0)
		{
			sprintf(str[j],"-> Exit");
		}else
		{
			sprintf(str[j],"   Exit");
		}
	}

	display_show(str[0], str[1], str[2], str[3]);
}

void display_showOptions(void)
{
	uint8_t i,j;
	char str[4][16] = {0};

	i = optionIndex;
	j = 0;
	while(strlen(ptrOptions[i].str) > 0 && j < 4)
	{
		if (i == optionIndex)
		{
			sprintf(str[j],"-> %s",ptrOptions[i].str);
		}else
		{
			sprintf(str[j],"   %s",ptrOptions[i].str);
		}
		j++;
		i++;
	}

	display_show(str[0], str[1], str[2], str[3]);
}

void display_forceMenuActive(void)
{
	menuActive = true;
}

void display_updateMenu(void)
{
		if (ptrOptions != NULL)
		{
			display_showOptions();
		}else
		{
			display_showMenu();
		}

	//handle push buttons
	if (GPIO_ReadInputDataBit(PIN_SW, PIN_SW3_ENTER) == 0 && (buttonState & 0x02) == 0)	//enter
	{
		buttonState |= 0x02;

		if (ptrMenu[menuIndex].func == NULL)
		{
			//exit pressed
			menuIndex = 0; //reset menu index
			menuActive = false;
			return;
		}

		if (ptrMenu[menuIndex].func != NULL)
		{
			if (ptrOptions != NULL)	//enter
			{
				char *ptrArgV[1];
				char str[25] = {0};
				ptrArgV[0] = str;
				sprintf(str,"%d",optionIndex);
				ptrMenu[menuIndex].func(1,ptrArgV);
				ptrOptions = NULL;	//ptrOptions=NULL
				optionIndex = 0;
			}else									//enter
			{
				int i;
				i = ptrMenu[menuIndex].func(0,NULL);
				if (ptrMenu[menuIndex].ptrOptions != NULL)
				{
					ptrOptions = ptrMenu[menuIndex].ptrOptions;	// enter ptrOptions=ptrMenu[menuIndex].ptrOptions
					optionIndex = i;
				}
			}
			return;
		}
	}
	//(buttonState & 0x01)==0
	if (GPIO_ReadInputDataBit(GPIO_JP, PIN_JP1) == 0 && (buttonState & 0x01) == 0)
	{
		buttonState |= 0x01;

		if (ptrOptions != NULL)
		{
			optionIndex++;
			if (strlen(ptrOptions[optionIndex].str) == 0)
			{
				optionIndex = 0;
			}
		} else
		{
			if (ptrMenu[menuIndex].func != NULL)
			{
				menuIndex++;
			} else
			{
				menuIndex = 0;
			}
		}

	}

	if (GPIO_ReadInputDataBit(GPIO_JP, PIN_JP1))
	{
		buttonState &= (~0x01);
	}

	if (GPIO_ReadInputDataBit(PIN_SW, PIN_SW3_ENTER))
	{
		buttonState &= (~0x02);
	}
}

//olde display
void display_process(void)
{
		if (false == menuActive || ptrMenu == NULL)
		{
			display_updateLCD();
		}
		else
		{
			display_updateMenu();
		}

	if (GPIO_ReadInputDataBit(GPIO_JP, PIN_JP3) == 0 && (buttonState & 0x04) == 0)	//exit
	{
		buttonState |= 0x04;
		menuActive = (bool)(!menuActive);
	}

	if (GPIO_ReadInputDataBit(GPIO_JP, PIN_JP3))
	{
		buttonState &= (~0x04);
	}
}

void display_updateLCD(void)
{
	char str[4][25] = {0};

// sprintf(str[0], "Increment PID");
//	switch(systemParams.controllerMode)
//	{
//		case CTRL_SIMPLE:
//			sprintf(str[0], "Simple PID");
//			break;

//		case CTRL_POS_PID:
//			sprintf(str[0], "Position PID");
//			break;

//		case CTRL_POS_VELOCITY_PID:
//			sprintf(str[0], "Velocity PID");
//			break;

//		default:
//			sprintf(str[0], "Error %u",systemParams.controllerMode);
//			break;
//	}
	int16_t speed_slow_rpm = speed_slow * (60u>>2) / (int16_t) (ANGLE_STEPS>>2);
	sprintf(str[0],"%3d rpm", (speed_slow_rpm <= 1 && speed_slow_rpm >= -1) ? 0 : speed_slow_rpm); 
	
	int32_t err = loopError;
	int32_t x, y, z;
	err = (err * 360 * 100) >> 16;
	z = err / 100;
	y = abs(err - (z * 100));
	sprintf(str[1],"%01ld.%02ld err", z,y);

	int64_t deg;
	deg = desiredLocation - zeroAngleOffset;
	deg = (deg * 225) / (ANGLE_STEPS >> 4);	//deg = (deg * 360 * 10) / (int32_t)ANGLE_STEPS;

	if (abs(deg) > 9999)
	{
		deg = deg / 1000;
	}

	x = deg / 10;
	y = abs(deg - (x * 10));

	display_show(str[0], str[1], str[2], str[3]);
}
