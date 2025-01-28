#define UNITY_INCLUDE_PRINT_FORMATTED
#include <unity.h>
#include "board.h"


// LED pin definition
#define VBAT_ADC_PIN       GPIO_PIN_0
#define VBAT_ADC_PORT      GPIOA

// ADC channel and port definition
#define ADC_CHANNEL   ADC_CHANNEL_0
#define ADC_PORT      GPIOA


void setUp(void) {
	adc_update_all();
}

void tearDown(void) {
    A4950_enable(false); //turn off even if test fails
    printf("\n");
    TEST_PRINTF("GetSupplyVoltage: %f V\n", GetSupplyVoltage());
    TEST_PRINTF("GetMotorVoltage: %f V\n", GetMotorVoltage());
    TEST_PRINTF("Get_PhaseA_Current: %f A\n", Get_PhaseA_Current());
    TEST_PRINTF("Get_PhaseB_Current: %f A\n", Get_PhaseB_Current());
    TEST_PRINTF("Temp idle: %f C\n", GetChipTemp());
}

#include "motor.h"
#include "delay.h"

static void test_normal_adc_values(void) {
    TEST_ASSERT_FLOAT_WITHIN(.05, 3.366, GetVDDA());
    TEST_ASSERT_FLOAT_WITHIN(.15, 12, GetSupplyVoltage());
    TEST_ASSERT_FLOAT_WITHIN(.2, GetSupplyVoltage()-0.1, GetMotorVoltage());
    TEST_ASSERT(GetSupplyVoltage() >= GetMotorVoltage()); //there should be voltage drop due to reverse polarity protection Schottky diode
    TEST_ASSERT_FLOAT_WITHIN(.1, 0, Get_PhaseA_Current());
    TEST_ASSERT_FLOAT_WITHIN(.1, 0, Get_PhaseB_Current());
    TEST_ASSERT_FLOAT_WITHIN(10, 20, GetChipTemp());
}



static void test_adc_values_under_load(void) {
    // TEST_IGNORE_MESSAGE("No motor ignore")
    A4950_enable(true);
    openloop_step(FULLSTEP_ELECTRIC_ANGLE*0, 3500);
    delay_ms(1000);

    adc_update_all(); 
    TEST_ASSERT_FLOAT_WITHIN(2, 10, GetSupplyVoltage());
    TEST_ASSERT_FLOAT_WITHIN(1, GetSupplyVoltage()-0.1, GetMotorVoltage());
    TEST_ASSERT(GetSupplyVoltage() > GetMotorVoltage()); //there should be voltage drop due to reverse polarity protection Schottky diode
    TEST_ASSERT_FLOAT_WITHIN(1, 3, Get_PhaseA_Current());
    TEST_ASSERT_FLOAT_WITHIN(.1, 0, Get_PhaseB_Current()); //todo proper current ADC accuracy
    TEST_ASSERT_FLOAT_WITHIN(10, 22, GetChipTemp());


    openloop_step(FULLSTEP_ELECTRIC_ANGLE*1, 3500);
    delay_ms(1000);

    adc_update_all(); 
    TEST_ASSERT_FLOAT_WITHIN(2, 10, GetSupplyVoltage());
    TEST_ASSERT_FLOAT_WITHIN(2, 10, GetMotorVoltage());
    TEST_ASSERT(GetSupplyVoltage() > GetMotorVoltage()); //there should be voltage drop due to reverse polarity protection Schottky diode
    TEST_ASSERT_FLOAT_WITHIN(.1, 0, Get_PhaseA_Current());
    TEST_ASSERT_FLOAT_WITHIN(1, 3, Get_PhaseB_Current()); //todo proper current ADC accuracy
    TEST_ASSERT_FLOAT_WITHIN(10, 24, GetChipTemp());

}


int main(void) {
    UNITY_BEGIN();
    board_init();
    RCC_ClocksTypeDef clks;
    RCC_GetClocksFreq(&clks);
    printf("ADC clock: %lu Hz\n", clks.ADCCLK_Frequency);
    printf("VDDA: %f V\n", GetVDDA());

    RUN_TEST(test_normal_adc_values);
    RUN_TEST(test_adc_values_under_load);
    delay_ms(3000);      //let the supply stabilize, if not Vmot might be above Vinn, due to inductance
    RUN_TEST(test_normal_adc_values);
    return UNITY_END();
}
