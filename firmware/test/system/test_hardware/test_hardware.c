#define UNITY_INCLUDE_PRINT_FORMATTED
#include <unity.h>
#include "board.h"
#include <math.h>
#include "motor.h"
#include "delay.h"
#include "utils.h"


//test conditions
#define supply_voltage 12.0f // power supply
#define phase_voltage 9.0f  // ensure not too high to not trigger current limit
#define cable_resistance 0.4f // ensure decent cabling
#define motor_phase_resistance 1.9f
#define ambient_temp 20.0f


void setUp(void) {
	adc_update_all();
}

void tearDown(void) {
    A4950_enable(false); //turn off even if test fails
    printf("\n");
    TEST_PRINTF("GetSupplyVoltage: %fV\n", GetSupplyVoltage());
    TEST_PRINTF("GetMotorVoltage: %fV\n", GetMotorVoltage());
    TEST_PRINTF("Get_PhaseA_Current: %fA\n", Get_PhaseA_Current());
    TEST_PRINTF("Get_PhaseB_Current: %fA\n", Get_PhaseB_Current());
    TEST_PRINTF("Temp idle: %fC\n", GetChipTemp());
    TEST_PRINTF("VDDA: %fV\n", GetVDDA());
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

float a4950_max_possible_current(void){
    const uint16_t I_RS_A4950_rat = RS_A4950 / ((uint16_t)Ohm_to_mOhm / I_RS_A4950_div); // I_RS_A4950_rat = 1
    float a4950_max = GetVDDA() * I_RS_A4950_rat; //current limit for 100% duty cycle
    TEST_PRINTF("a4950_max_possible_current: %f A\n", a4950_max);
    return a4950_max;
}

static float supply_volt_load(float supply_volt, float current){
    float cabling_voltage_drop = current * cable_resistance;
    float supply_volt_load = supply_volt - cabling_voltage_drop;
    // TEST_PRINTF("supply_volt_load: %f V\n", supply_volt_load);
    return supply_volt_load;
}

static float motor_current(float motor_volt){
    return fmin(motor_volt/motor_phase_resistance, a4950_max_possible_current());
}

static float motor_volt_load(float supply_volt, float current){
    float schottky_forward_voltage = (0.3f + (current * 0.05f));// approximate instantaneous Forward Voltage (V) chart from datasheet: https://jlcpcb.com/partdetail/MDD_Microdiode_Electronics-SS54/C22452
    // TEST_PRINTF("schottky_voltage_drop: %f V\n", schottky_forward_voltage);
    float motor_volt_load = supply_volt - schottky_forward_voltage;
    // TEST_PRINTF("motor_volt_load: %f V\n", motor_volt_load);
    return motor_volt_load;
}

static void test_adc_values_under_load(void) {
    float expected_current = motor_current(phase_voltage);
    TEST_PRINTF("expected_current of a single phase: %f A\n", expected_current);
    // estimate supply voltage under load
    float supply_volt_load_expected = supply_volt_load(GetSupplyVoltage(), expected_current);

    A4950_enable(true);
    for (int phase = 0; phase < 2; phase++){ // test both motor phases
        TEST_PRINTF("Testing phase %d\n", phase);
        float init_temp = GetChipTemp();
        int16_t phaseV = phase_voltage * 1000; // request half to make sure we don't trigger current limit for most motors
        if (phase > 1){
            phaseV = -phaseV; //test negative voltage
        }
        for (int i = 0; i < 100; i++){
            // phaseV = GetSupplyVoltage_mV()/2; // half duty cycle will produce 2x power supply current
            phase_voltage_command(phase%2==0 ? phaseV : 0, phase%2==1 ?  phaseV : 0, I_MAX_A4950);
            // phase_voltage_command(phaseV,  phaseV, I_MAX_A4950);
            delay_ms(10);
            adc_update_all(); 
        }

        TEST_ASSERT_FLOAT_WITHIN(0.4, supply_volt_load_expected, GetSupplyVoltage());
        TEST_ASSERT(GetSupplyVoltage() > GetMotorVoltage()); // sanity check - there should be voltage drop due to reverse polarity protection Schottky diode

        // estimate motor voltage under load
        float motor_volt_load_expected = motor_volt_load(GetSupplyVoltage(), expected_current);
        TEST_PRINTF("motor_volt_load_expected: %f V\n", motor_volt_load_expected);
        TEST_ASSERT_FLOAT_WITHIN(0.2, motor_volt_load_expected, GetMotorVoltage()); // Schottky voltage drop

        TEST_PRINTF("PhaseA_Current: %f A\n", Get_PhaseA_Current());
        TEST_PRINTF("PhaseB_Current: %f A\n", Get_PhaseB_Current());
        TEST_ASSERT_FLOAT_WITHIN(1, expected_current, phase ? Get_PhaseB_Current() : Get_PhaseA_Current()); //todo proper current ADC accuracy
        TEST_ASSERT_FLOAT_WITHIN(.1, 0, phase ? Get_PhaseA_Current() : Get_PhaseB_Current());

        TEST_ASSERT_FLOAT_WITHIN(2, init_temp += 1.0f, GetChipTemp()); //expected to heat up during the test
    }
}


int main(void) {
    UNITY_BEGIN();
    board_init();
    adc_update_all(); // throw away first measurement

    RCC_ClocksTypeDef clks;
    RCC_GetClocksFreq(&clks);
    printf("ADC clock: %lu Hz\n", clks.ADCCLK_Frequency);
    printf("VDDA: %f V\n", GetVDDA());

    RUN_TEST(test_normal_adc_values);
    RUN_TEST(test_adc_values_under_load);
    // delay_ms(3000);      //let the supply stabilize, if not Vmot might be above Vin
    // RUN_TEST(test_normal_adc_values); //test return to normal
    return UNITY_END();
}
