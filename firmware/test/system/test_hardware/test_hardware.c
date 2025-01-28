#define UNITY_INCLUDE_PRINT_FORMATTED
#include <unity.h>
#include "board.h"
#include <math.h>
#include "motor.h"
#include "delay.h"
#include "utils.h"


//test conditions
#define supply_voltage 12.0f
#define cable_resistance 0.4f // ensure decent cabling
#define motor_phase_resistance 3.0f
#define ambient_temp 20.0f


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



static void test_normal_adc_values(void) {
    TEST_ASSERT_FLOAT_WITHIN(.05, 3.366, GetVDDA());
    TEST_ASSERT_FLOAT_WITHIN(.15, supply_voltage, GetSupplyVoltage());
    TEST_ASSERT_FLOAT_WITHIN(.2, GetSupplyVoltage()-0.1, GetMotorVoltage());
    TEST_ASSERT(GetSupplyVoltage() >= GetMotorVoltage()); //there should be voltage drop due to reverse polarity protection Schottky diode
    TEST_ASSERT_FLOAT_WITHIN(.1, 0, Get_PhaseA_Current());
    TEST_ASSERT_FLOAT_WITHIN(.1, 0, Get_PhaseB_Current());
    TEST_ASSERT_FLOAT_WITHIN(10, ambient_temp, GetChipTemp());
    
}
static float a4950_max;
void a4950_max_current(void){
    const uint16_t I_RS_A4950_rat = RS_A4950 / ((uint16_t)Ohm_to_mOhm / I_RS_A4950_div); // I_RS_A4950_rat = 1
    a4950_max = GetVDDA() * I_RS_A4950_rat; //current limit for 100% duty cycle
    TEST_PRINTF("a4950_max_current: %f A\n", a4950_max);
}

static float supply_volt_load(float supply_volt, float current){
    float cabling_voltage_drop = current * cable_resistance;
    float supply_volt_load = supply_volt - cabling_voltage_drop;
    TEST_PRINTF("supply_volt_load: %f V\n", supply_volt_load);
    return supply_volt_load;
}

static float motor_current(float motor_volt){
    return fmin(motor_volt/motor_phase_resistance, a4950_max);
}

static float motor_volt_load(float supply_volt, float current){
    float schottky_forward_voltage = (0.3f + (current * 0.05f));// approximate instantaneous Forward Voltage (V) chart from datasheet: https://jlcpcb.com/partdetail/MDD_Microdiode_Electronics-SS54/C22452
    TEST_PRINTF("schottky_voltage_drop: %f V\n", schottky_forward_voltage);
    float motor_volt_load = supply_volt - schottky_forward_voltage;
    TEST_PRINTF("motor_volt_load: %f V\n", motor_volt_load);
    return motor_volt_load;
}

static void test_adc_values_under_load(void) {
    a4950_max_current();
    float supply_volt_init = GetSupplyVoltage(); // supply voltage when no load
    float supply_volt_load_expected = supply_volt_init;
    float motor_volt_load_expected = supply_volt_load_expected;
    float expected_current;
    // estimate supply voltage under load
    for (int i = 0; i < 10; i++){
        expected_current = motor_current(motor_volt_load_expected);
        supply_volt_load_expected = supply_volt_load(supply_volt_init, expected_current);
        motor_volt_load_expected = motor_volt_load(supply_volt_load_expected, expected_current);
    }
    TEST_PRINTF("motor_volt_load_expected: %f V\n", motor_volt_load_expected);


    A4950_enable(true);
    for (int phase = 0; phase < 1; phase++){ // test both motor phases
        float init_temp = GetChipTemp();
        openloop_step(FULLSTEP_ELECTRIC_ANGLE*phase, I_MAX_A4950*2); //make sure pwm is 100% - for each phase
        delay_ms(1000);

        adc_update_all(); 
        TEST_ASSERT_FLOAT_WITHIN(0.1, supply_volt_load_expected, GetSupplyVoltage());
        TEST_ASSERT(GetSupplyVoltage() > GetMotorVoltage()); // sanity check - there should be voltage drop due to reverse polarity protection Schottky diode
        // estimate motor voltage under load better
        for (int i = 0; i < 6; i++){
            expected_current = motor_current(motor_volt_load_expected);
            motor_volt_load_expected = motor_volt_load(GetSupplyVoltage(), expected_current);
        }
        TEST_PRINTF("motor_volt_load_expected: %f V\n", motor_volt_load_expected);
        TEST_ASSERT_FLOAT_WITHIN(0.1, motor_volt_load_expected, GetMotorVoltage()); // Schottky voltage drop
        // estimate phase current better
        expected_current = motor_current(GetSupplyVoltage());
        TEST_PRINTF("expected_current: %f A\n", expected_current);
        TEST_ASSERT_FLOAT_WITHIN(1, expected_current, phase ? Get_PhaseB_Current() : Get_PhaseA_Current()); //todo proper current ADC accuracy
        TEST_ASSERT_FLOAT_WITHIN(.1, 0, phase ? Get_PhaseA_Current() : Get_PhaseB_Current());

        TEST_ASSERT_FLOAT_WITHIN(1, init_temp += 2.0f, GetChipTemp()); //expected to heat up during the test
    }
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
    delay_ms(3000);      //let the supply stabilize, if not Vmot might be above Vin
    RUN_TEST(test_normal_adc_values); //test return to normal
    return UNITY_END();
}
