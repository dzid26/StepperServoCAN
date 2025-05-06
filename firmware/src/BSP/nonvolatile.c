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
#include "upgrade.h"

volatile MotorParams_t liveMotorParams;
volatile SystemParams_t liveSystemParams;
volatile PID_t pPID; //positional current based PID control parameters
volatile PID_t vPID; //velocity PID control parameters

nvm_t nvmMirror;
volatile uint32_t NVM_startAddress = PARAMETERS_FLASH_ADDR;

// Find nvm data actual address
void nonvolatile_begin(void)
{
	uint32_t i = ((FLASH_PAGE_SIZE / NONVOLATILE_STEPS) - 1); //(1024/62) = 16(0~15)
	
	NVM_startAddress = PARAMETERS_FLASH_ADDR;

	//search for the beginning of the last parameters structure segment (wear leveling)	
	for(i=((FLASH_PAGE_SIZE / NONVOLATILE_STEPS) - 1); i > 0; i--)
	{
		if( Flash_readHalfWord( (PARAMETERS_FLASH_ADDR + (i * NONVOLATILE_STEPS)) ) != invalid )
		{
			NVM_startAddress = (PARAMETERS_FLASH_ADDR + (i * NONVOLATILE_STEPS));
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

//NVM mirror - buffers NVM read/write
static void nvmMirrorInRam(void){
	//copy nvm from flash to ram
	nvmMirror = *(nvm_t*)NVM_startAddress; // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
}

//Check if empty (invalid)
bool nvmFlashCheck(uint32_t address, size_t n)
{
	uint32_t i;
	for(i=0; i < n; i++)
	{
		if( Flash_readHalfWord( address + (i * 2) ) != invalid )
			return false;
	}
	return true;
}


//currently only used once - after first boot
void nvmWriteConfParms(void){
	nvm_t* ptr_nvmMirror = &nvmMirror;
	ptr_nvmMirror->parametersValid  = valid;
	
	bool state = motion_task_isr_enabled;
	Motion_task_disable();
	//wear leveling
	if(Flash_readHalfWord(NVM_startAddress) != invalid && ((NVM_startAddress + NONVOLATILE_STEPS) < (PARAMETERS_FLASH_ADDR + FLASH_PAGE_SIZE)))
	{
		NVM_startAddress += NONVOLATILE_STEPS;
		
		while(nvmFlashCheck(NVM_startAddress, sizeof(nvm_t)/2U) == false )
		{
			if( (NVM_startAddress + NONVOLATILE_STEPS) < (PARAMETERS_FLASH_ADDR + FLASH_PAGE_SIZE))
			{
				NVM_startAddress += NONVOLATILE_STEPS;
			}
			else
			{
				NVM_startAddress = PARAMETERS_FLASH_ADDR;
				Flash_ProgramPage(NVM_startAddress, (uint16_t*)ptr_nvmMirror, (sizeof(nvm_t)/2U));
				return;
			}
		}
		Flash_ProgramSize(NVM_startAddress, (uint16_t*)ptr_nvmMirror, (sizeof(nvm_t)/2U));
	}
	else 
	{
		NVM_startAddress = PARAMETERS_FLASH_ADDR;
		Flash_ProgramPage(NVM_startAddress, (uint16_t*)ptr_nvmMirror, (sizeof(nvm_t)/2U));
	}

	nvmMirrorInRam();
	
	if (state) {
		Motion_task_enable();	
	}
	return;
}

//parameters first boot defaults and restore on corruption
void validateAndInitNVMParams(void)
{
	bool save_nvm = false;

	nvmMirrorInRam();

	// load defaults on first boot
	if (nvmMirror.parametersValid != valid){ //systemParams invalid
		nvmMirror.systemParams.fw_version = VERSION; // also set by app_upgrade_begin()
		nvmMirror.systemParams.controllerMode = CTRL_TORQUE;  //unused
		nvmMirror.systemParams.dirRotation = CCW_ROTATION;
		nvmMirror.systemParams.errorLimit = 0U;  //unused
		nvmMirror.systemParams.errorPinMode = ERROR_PIN_MODE_ACTIVE_LOW_ENABLE;  //default to !enable pin

		nvmMirror.motorParams.invertedPhase = false;
		nvmMirror.motorParams.fullStepsPerRotation = FULLSTEPS_NA; //it will be detected along with invertedPhase

		//the motor parameters are later checked in the stepper_controller code
		// as that there we can auto set much of them.

		save_nvm = true;
	}

	app_upgrade_begin(); // handles eeprom manipulation between versions if necessary

	if(NVM_SAVE_DEFAULT_PID_PARAMS || (nvmMirror.parametersValid != valid)) {
		nvmMirror.pPID.Kp = .5f;  nvmMirror.pPID.Ki = .0002f;  nvmMirror.pPID.Kd = 1.0f;  //range: 0-7.99 when CTRL_PID_SCALING=4096
		nvmMirror.vPID.Kp = 2.0f;   nvmMirror.vPID.Ki = 1.0f; 	 nvmMirror.vPID.Kd = 1.0f;
		save_nvm = true;
	}

	if (NVM_SET_ENABLE_PIN_MODE >= 0) {
		nvmMirror.systemParams.errorPinMode = NVM_SET_ENABLE_PIN_MODE;
		save_nvm = true;
	}

	if (NVM_SET_DIR_ROTATION >= 0) {
		nvmMirror.systemParams.dirRotation = NVM_SET_DIR_ROTATION;
		save_nvm = true;
	}

	if(NVM_SET_MOTOR_STEPS >= 0) {
		nvmMirror.motorParams.fullStepsPerRotation = NVM_SET_MOTOR_STEPS;
		save_nvm = true;
	}

	if(NVM_SET_MOTOR_PHASE_ORIENTATION >= 0) {
		nvmMirror.motorParams.invertedPhase = NVM_SET_MOTOR_PHASE_ORIENTATION;
		save_nvm = true;
	}
	
	
	if(save_nvm){
		nvmWriteConfParms(); //save defaults
	}
}
