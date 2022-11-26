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

#include "nonvolatile.h"
#include "board.h"
#include "stepper_controller.h"
#include "encoder.h"

volatile MotorParams_t motorParams;
volatile SystemParams_t systemParams;
volatile PID_t sPID; //simple control loop PID parameters
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

nvm_t nvmParams = {0};
volatile uint32_t NVM_address = PARAMETERS_FLASH_ADDR;

//NVM_address
void nonvolatile_begin(void)
{
	uint32_t i = ((FLASH_PAGE_SIZE / NONVOLATILE_STEPS) - 1); //(1024/62) = 16(0~15)
	
	NVM_address = PARAMETERS_FLASH_ADDR;
	
	for(i=((FLASH_PAGE_SIZE / NONVOLATILE_STEPS) - 1); i > 0; i--)
	{
		if( Flash_readHalfWord( (PARAMETERS_FLASH_ADDR + (i * NONVOLATILE_STEPS)) ) != invalid )
		{
			NVM_address = (PARAMETERS_FLASH_ADDR + (i * NONVOLATILE_STEPS));
			return;
		}
	}
}

void nvmWriteCalTable(void *ptrData)
{
	bool state = motion_task_isr_enabled;
	Motion_task_disable(); 
	
	Flash_ProgramPage(CALIBRATION_FLASH_ADDR, ptrData, (sizeof(FlashCalData_t)/2U));
	
	if (state) {
		Motion_task_enable();
	}
}

void nvmWriteConfParms(nvm_t* ptrNVM)
{		
	bool state = motion_task_isr_enabled;
	Motion_task_disable();
	
	ptrNVM->motorParams.parametersValid  = valid;
	ptrNVM->SystemParams.parametersValid = valid;
	
	if(Flash_readHalfWord(NVM_address) != invalid && ((NVM_address + NONVOLATILE_STEPS) < (PARAMETERS_FLASH_ADDR + FLASH_PAGE_SIZE)))
	{
		NVM_address += NONVOLATILE_STEPS;
		
		while( Flash_checknvmFlash(NVM_address, sizeof(nvm_t)/2U) == false )
		{																													 
			if( (NVM_address + NONVOLATILE_STEPS) < (PARAMETERS_FLASH_ADDR + FLASH_PAGE_SIZE))
			{
				NVM_address += NONVOLATILE_STEPS;
			}
			else
			{
				NVM_address = PARAMETERS_FLASH_ADDR;
				Flash_ProgramPage(NVM_address, (uint16_t*)ptrNVM, (sizeof(nvm_t)/2U));
				return;
			}
		}
		Flash_ProgramSize(NVM_address, (uint16_t*)ptrNVM, (sizeof(nvm_t)/2U));
	}
	else 
	{
		NVM_address = PARAMETERS_FLASH_ADDR;
		Flash_ProgramPage(NVM_address, (uint16_t*)ptrNVM, (sizeof(nvm_t)/2U));
	}
	
	motorParams = NVM->motorParams; //update motorParams
	systemParams = NVM->SystemParams; //update systemParams
	
	if (state) {
		Motion_task_enable();	
	}
	return;
}

//check the NVM and set to defaults if there is any
void validateAndInitNVMParams(void)
{
	if (NVM->SystemParams.parametersValid != valid) //SystemParams invalid
	{
		nvmParams.sPID.Kp = .1;  nvmParams.sPID.Ki = .008;  nvmParams.sPID.Kd = 0.2;
		nvmParams.pPID.Kp = 1.0;  nvmParams.pPID.Ki = 0.0; 	  nvmParams.pPID.Kd = 0.0;
		nvmParams.vPID.Kp = 2.0;  nvmParams.vPID.Ki = 1.0; 	  nvmParams.vPID.Kd = 1.0;

		nvmParams.SystemParams.microsteps = 256; //unused
		nvmParams.SystemParams.controllerMode = CTRL_SIMPLE;  //unused
		nvmParams.SystemParams.dirRotation = CCW_ROTATION;
		nvmParams.SystemParams.errorLimit = (int32_t)DEGREES_TO_ANGLERAW(1.8);  //unused
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
