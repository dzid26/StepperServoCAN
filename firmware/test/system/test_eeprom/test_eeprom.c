#define UNITY_INCLUDE_PRINT_FORMATTED
#include <unity.h>

#include "nonvolatile.h"
#include "APP/upgrade.h" //reference from /src/

void setUp(void) {
}

void tearDown(void) {
}

static void test_version(void) {
    TEST_PRINTF("This firmware version: %d", read_current_fw_version());
    TEST_PRINTF("Stored firmware version: %d", read_previous_fw_version());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(read_current_fw_version(), read_previous_fw_version());
    app_upgrade_begin();
    TEST_ASSERT_EQUAL_UINT16(read_current_fw_version(), read_previous_fw_version());
    TEST_ASSERT(true);
}

static void test_nvm_mirror_write(void){    
    const nvm_t* nvmFlash = (nvm_t*)NVM_startAddress; //pointer to the structure in the flash

    nvmMirror.motorParams.parametersValid = invalid;
    nvmMirror.systemParams.parametersValid = invalid;
    nvmWriteConfParms(); //after nvmWriteConfParms, invalid should be changed to valid
    TEST_ASSERT(nvmFlash->systemParams.parametersValid == valid);
    TEST_ASSERT(nvmFlash->motorParams.parametersValid == valid);
    TEST_ASSERT(nvmMirror.systemParams.parametersValid == valid);
    TEST_ASSERT(nvmMirror.motorParams.parametersValid == valid);
    TEST_ASSERT(memcmp(nvmFlash, &nvmMirror, sizeof(nvm_t)) == 0); // compare all underlying data

}

int main(void) {
    UNITY_BEGIN();

    // board_init();
    nonvolatile_begin();
    validateAndInitNVMParams();
    
    RUN_TEST(test_version);
    RUN_TEST(test_nvm_mirror_write);
    
    return UNITY_END();
}
