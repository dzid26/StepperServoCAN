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
	
	save_current_fw_version();
}