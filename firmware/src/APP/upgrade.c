#include "upgrade.h"
#include "nonvolatile.h"

#include "calibration.h"
#include "encoder.h"

uint16_t read_previous_fw_version(void){
	uint16_t version = nvmMirror.systemParams.fw_version;
	return version;
}

uint16_t read_current_fw_version(void){
	return VERSION;
}

static void save_current_fw_version(void){
	if(read_current_fw_version() != read_previous_fw_version()){
		nvmMirror.systemParams.fw_version = VERSION;
		nvmWriteConfParms(); //write default parameters
	}
}



void app_upgrade_begin(void){
	//upgrade v2 -> v3
	if((read_previous_fw_version() < 3000U) && (read_current_fw_version() >= 3000U)){
		// upgrade angle cals to new voltage-control that uses different uses canonical Parke transformation
		FlashCalData_t updated_cal;
		updated_cal.status = nvmFlashCalData->status; 	// cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; ++i ){
			updated_cal.FlashCalData[i] = nvmFlashCalData->FlashCalData[i] + (uint16_t)(ANGLE_STEPS / nvmMirror.motorParams.fullStepsPerRotation); // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		}
		nvmWriteCalTable(&updated_cal);
	}

	//downgrade v3 -> v2
	if((read_previous_fw_version() >= 3000U) && (read_current_fw_version() == 2999U)){
		// 0.2 -> 0.3
		// upgrade angle cals to new voltage-control that uses different uses canonical Parke transformation
		FlashCalData_t updated_cal;
		updated_cal.status = nvmFlashCalData->status; 	// cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; ++i ){
			updated_cal.FlashCalData[i] = nvmFlashCalData->FlashCalData[i] - (uint16_t)(ANGLE_STEPS / nvmMirror.motorParams.fullStepsPerRotation); // cppcheck-suppress  misra-c2012-11.4 - loading values from mapped flash structure
		}
		nvmWriteCalTable(&updated_cal);
	}
	
	save_current_fw_version();
}