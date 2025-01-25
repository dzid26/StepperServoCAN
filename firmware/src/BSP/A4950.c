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

#include "A4950.h"

#include "stepper_controller.h"
#include "nonvolatile.h"
#include "board.h"
#include "utils.h"

#define I_RS_A4950_div     (1000U/10U) //mOhm to Ohm and 10x multiplier


// phase lead due to DAC low pass filter C=uF, R=1k; phase = -atan(2pi*f*R*C)  
// generatePhaseLeadTable.py
#define PHASE_LEAD_MAX_SPEED  250u //revs/s
const uint16_t dacPhaseLead[PHASE_LEAD_MAX_SPEED] = {
	0,   5,  10,  15,  20,  25,  30,  35,  40,  45,  49,  54,  58,
	63,  67,  72,  76,  80,  84,  87,  91,  95,  98, 102, 105, 108,
	111, 114, 117, 120, 123, 126, 128, 131, 133, 136, 138, 140, 142,
	144, 146, 148, 150, 152, 154, 155, 157, 159, 160, 162, 163, 165,
	166, 168, 169, 170, 172, 173, 174, 175, 176, 177, 178, 180, 181,
	182, 183, 183, 184, 185, 186, 187, 188, 189, 190, 190, 191, 192,
	193, 193, 194, 195, 195, 196, 197, 197, 198, 199, 199, 200, 200,
	201, 201, 202, 202, 203, 204, 204, 205, 205, 205, 206, 206, 207, //100 rev/s
	207, 208, 208, 209, 209, 209, 210, 210, 211, 211, 211, 212, 212,
	212, 213, 213, 213, 214, 214, 214, 215, 215, 215, 216, 216, 216,
	217, 217, 217, 217, 218, 218, 218, 218, 219, 219, 219, 219, 220,
	220, 220, 220, 221, 221, 221, 221, 222, 222, 222, 222, 222, 223,
	223, 223, 223, 223, 224, 224, 224, 224, 224, 225, 225, 225, 225,
	225, 225, 226, 226, 226, 226, 226, 226, 227, 227, 227, 227, 227,
	227, 228, 228, 228, 228, 228, 228, 228, 229, 229, 229, 229, 229,
	229, 229, 229, 230, 230, 230, 230, 230, 230, 230, 230, 231, 231,
	231, 231, 231, 231, 231, 231, 232, 232, 232, 232, 232, 232, 232,
	232, 232, 232, 233, 233, 233, 233, 233, 233, 233, 233, 233, 233,
	234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 235, 235,
	235, 235, 235};

volatile bool driverEnabled = false;

/** 
 * Selects drive direction in current control mode
 * Compatible with GPIO_Mode_Out_PP and GPIO_Mode_AF_PP (timer) pin configurations
**/
inline static void bridgeA(int state)
{
	//Make sure the PIN_A4950_INs timer is counting to 1 to emulate GPIO
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MIN); //Count to 1. This is to use timer output as a gpio, because reconfiguring the pins to gpio online with CLR MODE register was annoying.
	if (state == 1) //Forward
	{	//User BRR BSRR reguisters to avoid ASSERT ehecution from HAL
		PIN_A4950->BSRR = PIN_A4950_IN1;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN1);		//IN1=1
		PIN_A4950->BRR = PIN_A4950_IN2;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN2);		//IN2=0
		TIM_SetCompare1(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare2(PWM_TIM, 0);
	}
	if (state == 0) //Reverse
	{
		PIN_A4950->BRR = PIN_A4950_IN1;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN1);		//IN1=0	
		PIN_A4950->BSRR = PIN_A4950_IN2;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN2);		//IN2=1
		TIM_SetCompare1(PWM_TIM, 0);
		TIM_SetCompare2(PWM_TIM, PWM_TIM_MIN+1);
	}
	if (state == 3) //Coast (off)
	{
		PIN_A4950->BRR = PIN_A4950_IN1;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN1);		//IN1=0
		PIN_A4950->BRR = PIN_A4950_IN2;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN2);		//IN2=0
		TIM_SetCompare1(PWM_TIM, 0);
		TIM_SetCompare2(PWM_TIM, 0);
	}
	if (state == 4) //brake
	{
		PIN_A4950->BSRR = PIN_A4950_IN1;		//GPIO_SetBits(PIN_A4950, PIN_A4950_IN1);	//IN1=1
		PIN_A4950->BSRR = PIN_A4950_IN2;		//GPIO_SetBits(PIN_A4950, PIN_A4950_IN2);	//IN2=1
		TIM_SetCompare1(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare2(PWM_TIM, PWM_TIM_MIN+1);
	}
}

/** 
 * Selects drive direction in current control mode
 * Compatible with GPIO_Mode_Out_PP and GPIO_Mode_AF_PP (timer) pin configurations
**/
inline static void bridgeB(int state)
{	
	// Make sure the PIN_A4950_INs timer is counting only to 1 to emulate GPIO
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MIN); //Count to 1. This is to use timer output as a gpio, because reconfiguring the pins to gpio online with CLR MODE register was annoying.
	if (state == 1) //Forward
	{
		PIN_A4950->BSRR = PIN_A4950_IN3;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN3);		//IN3=1
		PIN_A4950->BRR = PIN_A4950_IN4;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN4);		//IN4=0
		TIM_SetCompare3(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare4(PWM_TIM, 0);
	}
	if (state == 0) //Reverse
	{
		PIN_A4950->BRR = PIN_A4950_IN3;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN3);		//IN3=0
		PIN_A4950->BSRR = PIN_A4950_IN4;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN4);		//IN4=1
		TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, PWM_TIM_MIN+1);
	}
	if (state == 3) //Coast (off)
	{
		PIN_A4950->BRR = PIN_A4950_IN3;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN3);		//IN3=0
		PIN_A4950->BRR = PIN_A4950_IN4;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN4);		//IN4=0
		TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, 0);
	}
	if (state == 4) //brake
	{
		PIN_A4950->BSRR = PIN_A4950_IN3;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN1);		//IN3=1
		PIN_A4950->BSRR = PIN_A4950_IN4;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN2);		//IN4=1
		TIM_SetCompare3(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare4(PWM_TIM, PWM_TIM_MIN+1);
	}
}

//return duty with a maximum value VREF_TIM_MAX
/**
 * @brief Calculates duty with a maximum value VREF_TIM_MAX
 * 
 * @param current 
 * @return uint16_t 
 */
static uint16_t current_to_Vref_duty(uint16_t current)
{
	const uint16_t I_RS_A4950_rat = RS_A4950/I_RS_A4950_div; //mOhm to Ohm and 10x multiplier
	uint16_t vref = current * I_RS_A4950_rat;

	//limit Vref to MCU voltage
	uint16_t mcu_volt = GetMcuVoltage_mV();
	if(vref > mcu_volt){
		vref = mcu_volt;
	}
	uint16_t vref_duty = (uint16_t)((uint32_t) vref * VREF_TIM_MAX / mcu_volt);
	return vref_duty;
}

//Set reference voltage for the current driver regulator using PWM and RC filter
inline static void set_curr(uint16_t curr_lim_A, uint16_t curr_lim_B)
{
	uint16_t vref12 = current_to_Vref_duty(curr_lim_A);
	uint16_t vref34 = current_to_Vref_duty(curr_lim_B);

	//VREF12,34 between 0 and VREF_TIM_MAX corrresponds to 0 and mcu_volt mVolts
	TIM_SetCompare2(VREF_TIM, vref12);
	TIM_SetCompare1(VREF_TIM, vref34);

}

/**
 * @brief Set the PWM bridgeA duty cycle
 * 
 * @param duty - value between zero and PWM_TIM_MAX
 * @param quadrant1or2
 */
static const bool slow_decay = true;
static void setPWM_bridgeA(uint16_t duty, bool quadrant1or2)
{
	//Make sure the PIN_A4950_INs have running timer
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MAX);

	uint16_t pwm_count = min(duty, PWM_TIM_MAX);

	//duty_A,B between 0 and PWM_MAX corresponds to 0 and v_mot
	//quadrant1or2,3or4 - determines which phase is used

	if (slow_decay) //slow decay, zero duty is brake (phase shorted to ground ob both ends)
	{	 //electric field quadrant
		if (quadrant1or2)
		{ 		//"forward slow decay"
			TIM_SetCompare1(PWM_TIM, PWM_TIM_MAX);
			TIM_SetCompare2(PWM_TIM, PWM_TIM_MAX - pwm_count);
		}else{	//"reverse slow decay"
			TIM_SetCompare1(PWM_TIM, PWM_TIM_MAX - pwm_count);
			TIM_SetCompare2(PWM_TIM, PWM_TIM_MAX);
		}
	}else{	//fast decay, zero duty is coast (phase is floatinmg on both ends)
		if (quadrant1or2)
		{ 		//"forward fast decay"
			TIM_SetCompare1(PWM_TIM, pwm_count);
			TIM_SetCompare2(PWM_TIM, 0);
		}else{	//"reverse fast decay"
			TIM_SetCompare1(PWM_TIM, 0);
			TIM_SetCompare2(PWM_TIM, pwm_count);
		}
	}
}

/**
 * @brief Set the PWM bridgeB duty cycle
 * 
 * @param duty - value between zero and PWM_TIM_MAX
 * @param quadrant3or4
 */
static void setPWM_bridgeB(uint16_t duty, bool quadrant3or4)
{
	//Make sure the PIN_A4950_INs are configured as PWM
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MAX);

	uint16_t pwm_count = min(duty, PWM_TIM_MAX);

	if (slow_decay)
	{	 //electric field quadrant
		if (quadrant3or4)
		{		//"forward slow decay"
			TIM_SetCompare3(PWM_TIM, PWM_TIM_MAX);
			TIM_SetCompare4(PWM_TIM, PWM_TIM_MAX - pwm_count);
		}else{	//"reverse slow decay"
			TIM_SetCompare3(PWM_TIM, PWM_TIM_MAX - pwm_count);
			TIM_SetCompare4(PWM_TIM, PWM_TIM_MAX);
		}
	}else{		
		if (quadrant3or4)
		{		//"forward fast decay
			TIM_SetCompare3(PWM_TIM, pwm_count);
			TIM_SetCompare4(PWM_TIM, 0);
		}else{	//"reverse fast decay"
			TIM_SetCompare3(PWM_TIM, 0);
			TIM_SetCompare4(PWM_TIM, pwm_count);
		}
	}
}

void A4950_enable(bool enable)
{
	if (enable == false)
	{
		set_curr(0,0); //turn current off

		bridgeA(3); //tri state bridge outputs
		bridgeB(3); //tri state bridge outputs
		TIM_CtrlPWMOutputs(PWM_TIM, DISABLE);
	}else{
		TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
	}
	driverEnabled = enable;
}


/**
 * @brief Current based phase activation
 * 
 * @param I_a - phase A requested current
 * @param I_b - phase B requested current
 * @param curr_lim - current limit applied to each phase
 */
void phase_current_command(int16_t I_a, int16_t I_b){
	if (driverEnabled == false){
		set_curr(0,0); 	//turn current off
		bridgeA(3); 	//tri state bridge outputs
		bridgeB(3); 	//tri state bridge outputs
	}else{
		set_curr(fastAbs(I_a), fastAbs(I_b));

		bridgeA((I_a > 0) ? 1 : 0);
		if(liveMotorParams.swapPhase){
			bridgeB((I_b > 0) ? 0 : 1);
		}else{
			bridgeB((I_b > 0) ? 1 : 0);
		}
	}
}

/**
 * @brief Voltage based phase activation with current limit
 * 
 * @param U_a - phase A requested voltage
 * @param U_b - phase B requested voltage
 * @param curr_lim - current limit applied to each phase
 */
void phase_voltage_command(int16_t U_a, int16_t U_b, uint16_t curr_lim){
	if (driverEnabled == false){
		set_curr(0,0); 	//turn current off
		bridgeA(3); 	//tri state bridge outputs
		bridgeB(3); 	//tri state bridge outputs
	}else{
		set_curr(curr_lim, curr_lim); 

		uint16_t U_in = GetMotorVoltage_mV();
		uint16_t duty_a = (uint16_t)(fastAbs(U_a) * PWM_TIM_MAX / U_in);
		uint16_t duty_b = (uint16_t)(fastAbs(U_b) * PWM_TIM_MAX / U_in);
		setPWM_bridgeA(duty_a, (U_a > 0)); //PWM12
		setPWM_bridgeB(duty_b, liveMotorParams.swapPhase ? (U_b < 0) : (U_b > 0)); //PWM34
	}
}