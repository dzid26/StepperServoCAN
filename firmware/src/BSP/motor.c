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

	// max practical phase voltage is U_lim*sqrt(2)
	int32_t a = ((((int32_t)cos * D) - ((int32_t)sin * Q)) / (int32_t)SINE_MAX);
	int32_t b = ((((int32_t)sin * D) + ((int32_t)cos * Q)) / (int32_t)SINE_MAX);

	//limit to int16_t (to -32V..32V)
	*A = (int16_t)(int32_t)clip(a, INT16_MIN, INT16_MAX);
	*B = (int16_t)(int32_t)clip(b, INT16_MIN, INT16_MAX);
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
	
	const int16_t angleSensLatency = SENS_DELAY;  //angle sensor delay - bigger value can result in higher speed (because it fakes field weakening), but can be detrimental to motor power and efficiency

	int16_t angleSpeedComp = (int16_t) (speed_slow * angleSensLatency / (int32_t) S_to_uS);

	//convert load angle to electrical angle domain (0-1023 full turn)
	uint16_t absoluteAngle = (uint16_t)(((uint32_t)(int32_t)(currentLocation + angleSpeedComp)) & ANGLE_MAX);
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

void base_speed_test(int16_t dir) {
	uint16_t electricAngle = calc_electric_angle(true);
	const uint16_t safety_torque_limit = 500U; //mA - should not interfere with reaching full speed

	int16_t U_lim = (int16_t)min(GetMotorVoltage_mV(), INT16_MAX);
	if (dir != 0) {
		voltage_commutation(electricAngle, clip(dir, -1, 1) * U_lim, 0, safety_torque_limit);
	}else{
		// freewheeling
		int32_t U_emf = (int32_t)((int64_t)motor_k_bemf * speed_slow / (int32_t)ANGLE_STEPS);
		int16_t U_emf_sat = (int16_t)clip(U_emf, -U_lim, U_lim);
		voltage_commutation(electricAngle, U_emf_sat, 0, safety_torque_limit);
	}
}

volatile int16_t I_q_est = 0;
volatile int16_t I_d_est = 0;
volatile int16_t U_q_sat = 0;
volatile int16_t U_d_sat = 0;

void field_oriented_control(int16_t current_target) {

	const bool volt_control = USE_VOLTAGE_CONTROL;

	int16_t current_actual;
	uint16_t electricAngle = calc_electric_angle(volt_control);

	int16_t I_q = current_target;
	int16_t I_d = 0;
	if(volt_control == true){
		uint16_t motor_rev_to_elec_rad = (uint16_t)((uint32_t)TWO_PI_X1024 * liveMotorParams.fullStepsPerRotation / 4U / 1024U); //typically 314 (or 628 for 0.9deg motor)
		// electrical angle per second in radians
		int32_t e_rad_s = (int32_t)((int64_t)motor_rev_to_elec_rad * speed_slow / (int32_t)ANGLE_STEPS);
		// DC link voltage:
		int16_t U_lim = (int16_t)min(GetMotorVoltage_mV(), INT16_MAX);

		// Iq, Id, Uq, Ud per FOC nomencluture
		// Qadrature axis (steady state): U_q = I_q*R + I_d*ω*L + U_emf
		int16_t U_IqR = (int16_t)((int32_t)I_q * phase_R / Ohm_to_mOhm);
		int32_t U_IdwL = (int32_t)((int64_t)I_d * e_rad_s * phase_L / H_to_uH);
		int32_t U_emf = (int32_t)((int64_t)motor_k_bemf * speed_slow / (int32_t)ANGLE_STEPS);
		int32_t U_q = U_IqR + U_IdwL + U_emf;
		U_q_sat = (int16_t)clip(U_q, -U_lim, U_lim);

		// estimate commanded Iq after saturations
		int16_t U_IdL_emf_sat = (int16_t)clip(U_emf + U_IdwL, -U_lim, U_lim);
		int16_t U_IqR_est = U_q_sat - U_IdL_emf_sat;
		I_q_est = (int16_t)((int32_t)U_IqR_est * Ohm_to_mOhm / phase_R);
		current_actual = I_q_est;

		// Direct Axis (steady state): U_d = I_d*R - I_q*ω*L
		int16_t U_IdR = (int16_t)((int32_t)I_d * phase_R / Ohm_to_mOhm);
		int32_t U_IqwL = -(int32_t)((int64_t)I_q_est * e_rad_s * phase_L / H_to_uH);
		int32_t U_d = U_IdR + U_IqwL;
		U_d_sat = (int16_t)(clip(U_d, -U_lim, U_lim));

		// estimate I_d after saturations
		int16_t U_IqL_sat = (int16_t)clip(U_IqwL, -U_lim, U_lim);
		int16_t U_IdR_est = U_d_sat - U_IqL_sat;
		I_d_est = (int16_t)((int32_t)U_IdR_est * Ohm_to_mOhm / phase_R);

		uint16_t magnitude = fastAbs(I_q_est) + fastAbs(I_d_est); // approx of sqrt(I_q^2 + I_d^2)
		voltage_commutation(electricAngle, U_q_sat, U_d_sat, magnitude);
	}else{
		current_commutation(electricAngle, I_q, I_d);
		current_actual = current_target; // simplification for higher speeds - //todo estimate or measure actual current

	}
	control_actual = current_actual;
}

