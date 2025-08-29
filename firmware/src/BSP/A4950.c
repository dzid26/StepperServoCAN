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

// phase lead due to DAC low pass filter C=uF, R=1k; phase = -atan(2pi*f*R*C)  
// generatePhaseLeadTable.py
const uint16_t dacPhaseLead[PHASE_LEAD_MAX_SPEED] = {
	0,   2,   4,   6,   9,  11,  13,  15,  18,  20,  22,  24,  26,
	29,  31,  33,  35,  38,  40,  42,  44,  46,  49,  51,  53,  55,
	57,  60,  62,  64,  66,  68,  70,  73,  75,  77,  79,  81,  83,
	85,  87,  90,  92,  94,  96,  98, 100, 102, 104, 106, 108, 110,
	112, 114, 116, 118, 120, 122, 124, 126, 128, 130, 131, 133, 135,
	137, 139, 141, 143, 145, 146, 148, 150, 152, 154, 155, 157, 159,
	161, 162, 164, 166, 168, 169, 171, 173, 174, 176, 178, 179, 181,
	182, 184, 186, 187, 189, 190, 192, 194, 195};

volatile bool driverEnabled = false;

/** 
 * H-bridge ON/OFF mode
 * Selects drive direction in current control mode
 * A4950_A_IN1 | A4950_A_IN2
**/
inline static void bridgeA(int state){
	// Make sure the PIN_A4950_INs timer is counting only to 1 to emulate GPIO
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MIN); //Count to 1. This is to use timer output as a gpio, because reconfiguring the pins to gpio online with CLR MODE register was annoying.
	if (state == 1){ //Forward
		TIM_SetCompare1(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare2(PWM_TIM, 0);
	}
	if (state == 0){ //Reverse
		TIM_SetCompare1(PWM_TIM, 0);
		TIM_SetCompare2(PWM_TIM, PWM_TIM_MIN+1);
	}
	if (state == 3){ //Coast (off)
		TIM_SetCompare1(PWM_TIM, 0);
		TIM_SetCompare2(PWM_TIM, 0);
	}
	if (state == 4){ //brake
		TIM_SetCompare1(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare2(PWM_TIM, PWM_TIM_MIN+1);
	}
}

/** 
 * H-bridge ON/OFF mode
 * Selects drive direction in current control mode
 * A4950_B_IN1 | A4950_B_IN2
**/
inline static void bridgeB(int state){
	// Make sure the PIN_A4950_INs timer is counting only to 1 to emulate GPIO
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MIN); //Count to 1. This is to use timer output as a gpio, because reconfiguring the pins to gpio online with CLR MODE register was annoying.
	if (state == 1){ //Forward
		TIM_SetCompare3(PWM_TIM, PWM_TIM_MIN+1);
		TIM_SetCompare4(PWM_TIM, 0);
	}
	if (state == 0){ //Reverse
		TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, PWM_TIM_MIN+1);
	}
	if (state == 3){ //Coast (off)
		TIM_SetCompare3(PWM_TIM, 0);
		TIM_SetCompare4(PWM_TIM, 0);
	}
	if (state == 4){ //brake
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
	const uint16_t I_RS_A4950_rat = RS_A4950 / ((uint16_t)Ohm_to_mOhm / I_RS_A4950_div);
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
 * @param duty - value between zero and PWM_TIM_MAX corresponds to 0 and v_mot
 * @param quadrant1or2 - determines phase polarity
 */
static const bool slow_decay = true;
static void setPWM_bridgeA(uint16_t duty, bool quadrant1or2){
	//Make sure the PIN_A4950_INs have running timer
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MAX);

	uint16_t pwm_count = min(duty, PWM_TIM_MAX);

	if (slow_decay){ //slow decay, zero duty is brake (phase shorted to ground ob both ends)
		//electric angle quadrant
		if (quadrant1or2){
			// "forward slow decay"
			TIM_SetCompare1(PWM_TIM, PWM_TIM_MAX);
			TIM_SetCompare2(PWM_TIM, PWM_TIM_MAX - pwm_count);
		}else{
			// "reverse slow decay"
			TIM_SetCompare1(PWM_TIM, PWM_TIM_MAX - pwm_count);
			TIM_SetCompare2(PWM_TIM, PWM_TIM_MAX);
		}
	}else{	// fast decay, zero duty is coast (phase is floatinmg on both ends)
		if (quadrant1or2){
			// "forward fast decay"
			TIM_SetCompare1(PWM_TIM, pwm_count);
			TIM_SetCompare2(PWM_TIM, 0);
		}else{
			// "reverse fast decay"
			TIM_SetCompare1(PWM_TIM, 0);
			TIM_SetCompare2(PWM_TIM, pwm_count);
		}
	}
}

/**
 * @brief Set the PWM bridgeB duty cycle
 * 
 * @param duty - value between zero and PWM_TIM_MAX corresponds to 0 and v_mot
 * @param quadrant3or4 - determines phase polarity
 */
static void setPWM_bridgeB(uint16_t duty, bool quadrant3or4){
	//Make sure the PIN_A4950_INs are configured as PWM
	TIM_SetAutoreload(PWM_TIM, PWM_TIM_MAX);

	uint16_t pwm_count = min(duty, PWM_TIM_MAX);

	if (slow_decay){
		//electric field quadrant
		if (quadrant3or4){
			// "forward slow decay"
			TIM_SetCompare3(PWM_TIM, PWM_TIM_MAX);
			TIM_SetCompare4(PWM_TIM, PWM_TIM_MAX - pwm_count);
		}else{
			// "reverse slow decay"
			TIM_SetCompare3(PWM_TIM, PWM_TIM_MAX - pwm_count);
			TIM_SetCompare4(PWM_TIM, PWM_TIM_MAX);
		}
	}else{		
		if (quadrant3or4){
			// "forward fast decay"
			TIM_SetCompare3(PWM_TIM, pwm_count);
			TIM_SetCompare4(PWM_TIM, 0);
		}else{
			// "reverse fast decay"
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
		if(liveMotorParams.invertedPhase){
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
		setPWM_bridgeB(duty_b, liveMotorParams.invertedPhase ? (U_b < 0) : (U_b > 0)); //PWM34
	}
}