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

#include "nonvolatile.h"
#include "board.h"

extern volatile MotorParams_t motorParams;
extern volatile SystemParams_t systemParams;

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

void nvmWriteFastCalTable(void *ptrData, uint16_t page)
{
	bool state = motion_task_isr_enabled;
	Motion_task_disable(); 
	
	Flash_ProgramPage(FASTCALIBRATION_FLASH_ADDR + ((uint32_t)FLASH_PAGE_SIZE * page), ptrData, FLASH_ROW_SIZE);
	
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
