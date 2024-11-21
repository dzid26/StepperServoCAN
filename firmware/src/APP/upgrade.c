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
	uint16_t version_upgrade;

	version_upgrade = 3002U;
	if((read_previous_fw_version() < version_upgrade) && (read_current_fw_version() >= version_upgrade)){  // cppcheck-suppress  knownConditionTrueFalse
		if (nvmMirror.pPID.Kp < 0.1f){
			nvmMirror.pPID.Kp = 0.5f;
		}
		if (nvmMirror.pPID.Kd < 0.1f){
			nvmMirror.pPID.Kd = 1.0f;
		}
		
		nvmWriteConfParms();
	}
	
	//upgrade v2 -> v3
	version_upgrade = 3000U;
	if((read_previous_fw_version() < version_upgrade) && (read_current_fw_version() >= version_upgrade)){  // cppcheck-suppress  knownConditionTrueFalse
		// upgrade angle cals to new voltage-control that uses different uses canonical Parke transformation
		FlashCalData_t updated_cal;
		updated_cal.status = nvmFlashCalData->status;
		for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; ++i ){
			updated_cal.FlashCalData[i] = nvmFlashCalData->FlashCalData[i] + (uint16_t)(ANGLE_STEPS / nvmMirror.motorParams.fullStepsPerRotation);
		}
		nvmWriteCalTable(&updated_cal);
	}

	//downgrade v3 -> v2
	version_upgrade = 3000U;
	if((read_previous_fw_version() >= version_upgrade) && (read_current_fw_version() == (version_upgrade-1U))){  // cppcheck-suppress  knownConditionTrueFalse
		// 0.2 -> 0.3
		// upgrade angle cals to new voltage-control that uses different uses canonical Parke transformation
		FlashCalData_t updated_cal;
		updated_cal.status = nvmFlashCalData->status;
		for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; ++i ){
			updated_cal.FlashCalData[i] = nvmFlashCalData->FlashCalData[i] - (uint16_t)(ANGLE_STEPS / nvmMirror.motorParams.fullStepsPerRotation);
		}
		nvmWriteCalTable(&updated_cal);
	}
	
	save_current_fw_version();
}