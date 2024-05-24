#include "motor.h"
#include "sine.h"
#include "actuator_config.h"
#include "stepper_controller.h"
#include "nonvolatile.h"
#include "encoder.h"
#include "utils.h"
#include "board.h"

static void inverse_park_transform(uint16_t elecAngle, int16_t Q, int16_t D, int16_t *A, int16_t *B){
	//calculate sine and cosine with ripple compensation
	int16_t sin = sine_ripple(elecAngle, anticogging_factor);
	int16_t cos = cosine_ripple(elecAngle, anticogging_factor);
	

	*A = (int16_t)((((int32_t)cos * D) - ((int32_t)sin * Q)) / (int32_t)SINE_MAX); //convert value with vref max corresponding to 3300mV
	*B = (int16_t)((((int32_t)sin * D) + ((int32_t)cos * Q)) / (int32_t)SINE_MAX); //convert value with vref max corresponding to 3300mV
}

/**
 * @brief Current commutation scheme
 * 
 * @param elecAngle - current angle in electrical degrees - 360 corresponds to SINE_STEPS
 * @param I_q - quadrature current command - corresponds to driving torque
 * @param I_d - direct current command - positive value corresponds to "holding torque" for open-loop stepping. Negative values can be used for field weakening
 */
static void current_commutation(uint16_t elecAngle, int16_t I_q, int16_t I_d)
{
	int16_t I_a = 0;
	int16_t I_b = 0;
	inverse_park_transform(elecAngle, I_q, I_d, &I_a, &I_b);
	
	phase_current_command(I_a, I_b);
}

/**
 * @brief Voltage commutation scheme
 * 
 * @param elecAngle - current angle in electrical degrees - 360 corresponds to SINE_STEPS
 * @param U_q - quadrature voltage command - corresponds to torque generating current command + BEMF compensation 
 * @param U_d - direct voltage command - corresponds to flux generating current + current lag compesantion
 */
static void voltage_commutation(uint16_t elecAngle, int16_t U_q, int16_t U_d, uint16_t curr_lim)
{	
	int16_t U_a = 0;
	int16_t U_b = 0;
	inverse_park_transform(elecAngle, U_q, U_d, &U_a, &U_b);
	
	phase_voltage_command(U_a, U_b, curr_lim);
}

void openloop_step(uint16_t elecAngleStep, uint16_t curr_tar){
	current_commutation(elecAngleStep, 0, (int16_t)curr_tar);
}


/**
 * @brief Converts absolut eangle to electric angle and applies delay compensations
 * @return multiples of electric angle
 */
static uint16_t calc_electric_angle(bool volt_control){
	
	int16_t angleSensLatency = 64u;  //angle sensor delay - bigger value can result in higher speed (because it fakes field weakening), but can be detrimental to motor power and efficiency

	int16_t angleSpeedComp = (int16_t) (speed_slow * angleSensLatency / (int32_t) S_to_uS);

	//convert load angle to electrical angle domain (0-1023 full turn)
	uint16_t absoluteAngle = (uint16_t)(((uint32_t)(int32_t)(currentLocation + angleSpeedComp)) & ANGLE_MAX); //add load angle to current location
	uint16_t electricAngle = absoluteAngle * liveMotorParams.fullStepsPerRotation * FULLSTEP_ELECTRIC_ANGLE / ANGLE_STEPS;

	//calculate microsteps phase lead for current control
	if (volt_control == false){
		uint16_t stepPhaseLead = 0;
		if (speed_slow > 0){
			stepPhaseLead = dacPhaseLead[min(((uint32_t) ( speed_slow) / ANGLE_STEPS),  PHASE_LEAD_MAX_SPEED - 1U)];
			electricAngle += stepPhaseLead;
		}else{
			stepPhaseLead = dacPhaseLead[min(((uint32_t) (-speed_slow) / ANGLE_STEPS),  PHASE_LEAD_MAX_SPEED - 1U)];
			electricAngle -= stepPhaseLead;
		}
	}
	return electricAngle % SINE_STEPS;
}

void field_oriented_control(int16_t current_target) 
{
	const bool volt_control = true;

	uint16_t electricAngle = calc_electric_angle(volt_control);

	int16_t I_q = current_target;
	if(volt_control == true){
		//Iq, Id, Uq, Ud per FOC nomencluture

		//Qadrature axis
		//U_q = I_q*R + U_emf
		int16_t U_IR = (int16_t)((int32_t)I_q * phase_R / Ohm_to_mOhm);
		int32_t U_emf = (int32_t)((int64_t)motor_k_bemf * speed_slow / (int32_t)ANGLE_STEPS);
		int32_t U_q = U_IR + U_emf;
		int16_t U_lim = (int16_t)min(GetMotorVoltage_mV(), INT16_MAX);
		int16_t U_emf_sat = (int16_t)clip(U_emf, -U_lim, U_lim);
		int16_t U_IR_sat = (int16_t)clip(U_IR, -U_lim - U_emf_sat, U_lim - U_emf_sat);
		U_IR_sat = (int16_t)clip(U_IR_sat, -U_lim, U_lim);
		int16_t U_q_sat = U_IR_sat + U_emf_sat;
		int16_t I_q_act = (int16_t)((int32_t)U_IR_sat * Ohm_to_mOhm / phase_R);
		control_actual = I_q_act;

		//Direct Axis
		//U_d = I_q*ω*Rl
		uint16_t motor_rev_to_elec_rad = (uint16_t)((uint32_t)TWO_PI_X1024 * liveMotorParams.fullStepsPerRotation / 4U / 1024U); //typically 314 (or 628 for 0.9deg motor)
		int32_t e_rad_s = (int32_t)((int64_t)motor_rev_to_elec_rad * speed_slow / (int32_t)ANGLE_STEPS);
		int32_t U_d = (int32_t)((int64_t)(-I_q_act) * e_rad_s * phase_L / H_to_uH) ; //Vd=Iq * ω*Rl

		int32_t const max_speed = (int32_t)ANGLE_STEPS * 64; //64rev/s -> 3840rpm
		int32_t base_speed = (int32_t)ANGLE_STEPS * U_lim / motor_k_bemf;//10rev/s@13V   - temporarily reduce by by the factor of two to be sure we start to trigger the weakening
		int32_t above_base_speed_delta = clip(((int32_t)fastAbs(speed_slow)-base_speed), 0, max_speed); 
		int32_t U_d_weakening = -(int32_t)((int64_t)above_base_speed_delta*U_lim/max_speed);
		if(speed_slow<0){
			U_d_weakening = -U_d_weakening;
		}
	
		int16_t I_d = (int16_t)((int32_t)U_d_weakening * Ohm_to_mOhm / phase_R);//backcalcualte - should have opposite sign to I_q
		int32_t U_q_I_dLw = (int32_t)((int64_t)I_d * e_rad_s * phase_L / H_to_uH);
		// U_q_sat += U_q_I_dLw;
		U_d +=  U_d_weakening;

		int16_t U_d_sat = (int16_t)(clip(U_d, -U_lim, U_lim));
		uint16_t magnitude = (uint16_t)((current_target > 0) ? I_q_act : -I_q_act); //abs
		voltage_commutation(electricAngle, U_q_sat, U_d_sat, magnitude);
	}else{
		current_commutation(electricAngle, I_q, 0);
		control_actual = (int16_t)clip(control, -MAX_CURRENT, MAX_CURRENT); // simplification - close to truth for low speeds

	}
}

