 /**
 * MKS SERVO42B
 * Copyright (c) 2020 Makerbase. 
 *
 * Based on nano_stepper project by Misfittech
 * Copyright (C) 2018  MisfitTech LLC.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "A4950.h"

#define MCU_VOUT 3300u //milliVolts

#include "stepper_controller.h"
extern volatile int32_t speed_slow;

// phase lead due to DAC low pass filter C=uF, R=1k; phase = -atan(2pi*f*R*C)  
// generatePhaseLeadTable.py
#define PHASE_LEAD_MAX_SPEED  250u //revs/s
static const uint16_t dacPhaseLead[PHASE_LEAD_MAX_SPEED] = {
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


extern volatile MotorParams_t motorParams;

volatile bool A4950_Enabled = false;

//phase 1
inline static void bridge1(int state)
{
	if (state == 0) //Forward
	{
		PIN_A4950->BSRR = PIN_A4950_IN1;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN1);		//IN1=1
		PIN_A4950->BRR = PIN_A4950_IN2;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN2);	//IN2=0
	}
	if (state == 1) //Reverse
	{
		PIN_A4950->BRR = PIN_A4950_IN1;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN1);	//IN1=0	
		PIN_A4950->BSRR = PIN_A4950_IN2;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN2);		//IN2=1	
	}
	if (state == 3) //Coast (off)
	{
		PIN_A4950->BRR = PIN_A4950_IN1;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN1);	//IN1=0
		PIN_A4950->BRR = PIN_A4950_IN2;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN2);	//IN2=0
	}
	if (state == 4) //brake
	{
		PIN_A4950->BSRR = PIN_A4950_IN1;		//GPIO_SetBits(PIN_A4950, PIN_A4950_IN1);	//IN1=0
		PIN_A4950->BSRR = PIN_A4950_IN2;		//GPIO_SetBits(PIN_A4950, PIN_A4950_IN2);	//IN2=0
	}
}

//phase 2
inline static void bridge2(int state)
{
	if (state == 0) //Forward
	{
		PIN_A4950->BSRR = PIN_A4950_IN3;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN3);		//IN3=1
		PIN_A4950->BRR = PIN_A4950_IN4;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN4);	//IN4=0
	}
	if (state == 1) //Reverse
	{
		PIN_A4950->BRR = PIN_A4950_IN3;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN3);	//IN3=0
		PIN_A4950->BSRR = PIN_A4950_IN4;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN4);		//IN4=1
	}
	if (state == 3) //Coast (off)
	{
		PIN_A4950->BRR = PIN_A4950_IN3;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN3);	//IN3=0
		PIN_A4950->BRR = PIN_A4950_IN4;		//GPIO_ResetBits(PIN_A4950, PIN_A4950_IN4);	//IN4=0
	}
	if (state == 4) //brake
	{
		PIN_A4950->BSRR = PIN_A4950_IN3;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN1);	//IN3=0
		PIN_A4950->BSRR = PIN_A4950_IN4;	//GPIO_SetBits(PIN_A4950, PIN_A4950_IN2);	//IN4=0
	}
}

//Vref
inline static void setVREF(uint16_t VREF12, uint16_t VREF34)
{
	//VREF_SCALER reduces PWM resolution by 2^VREF_SCALER,
	//but also increasesPWM freq - needed for low pass filter to effectively filter V reference)
	TIM_SetCompare3(VREF_TIM, VREF12>>VREF_SCALER);
	TIM_SetCompare4(VREF_TIM, VREF34>>VREF_SCALER);
}


void A4950_enable(bool enable)
{
	A4950_Enabled = enable;
	
	if (A4950_Enabled == false)
	{
		setVREF(0,0); //turn current off
		bridge1(3); //tri state bridge outputs
		bridge2(3); //tri state bridge outputs
	}
}

// this is precise move and modulo of A4950_STEP_MICROSTEPS is a full step.
// stepAngle is in A4950_STEP_MICROSTEPS units..
// The A4950 has no idea where the motor is, so the calling function has to
// tell the A4950 what phase to drive motor coils.
// A4950_STEP_MICROSTEPS is 256 by default so stepAngle of 1024 is 360 degrees
// Note you can only move up to +/-A4950_STEP_MICROSTEPS from where you
// currently are.

volatile uint16_t vrefY;
volatile uint16_t vrefX;
int32_t A4950_move(uint16_t stepAngle, uint16_t mA) //256 stepAngle is 90 electrical degrees
{
	uint16_t elecAngleStep;
	int16_t sin;
	int16_t cos;
	
	uint16_t stepPhaseLead = dacPhaseLead[min((uint32_t) fastAbs(speed_slow) / ANGLE_STEPS, PHASE_LEAD_MAX_SPEED)];
	if (speed_slow > 0){
		elecAngleStep = stepAngle + stepPhaseLead;
	}else{
		elecAngleStep = stepAngle - stepPhaseLead;	
	}

	//modulo operator of 2^N implemented as a bitmask of 2^N-1
	elecAngleStep = elecAngleStep & (SINE_STEPS-1u);
	
	if (A4950_Enabled == false)
	{
		setVREF(0,0); 	//turn current off
		bridge1(3); 	//tri state bridge outputs
		bridge2(3); 	//tri state bridge outputs
		return 0;
	}

	//calculate the sine and cosine of our elecAngleStep
	sin = sine(elecAngleStep);
	cos = cosine(elecAngleStep);

	//calculate the sine and cosine of our elecAngleStep - lumped park transform - FOC Q and D force vectors
	vrefX = (uint16_t)((mA * (uint16_t) fastAbs(cos)) / MCU_VOUT); //convert value into DAC scaled to 3300mV max
	vrefY = (uint16_t)((mA * (uint16_t) fastAbs(sin)) / MCU_VOUT);

	setVREF(vrefY,vrefX); //VREF12	VREF34

	if (sin > 0)
	{
		bridge1(1);
	}else
	{
		bridge1(0);
	}
	if (cos > 0)
	{	//reverse coils actuatoion if phases are swapped or reverse direction is selected
		bridge2(motorParams.motorWiring ? 1u : 0u); 
	}else
	{
		bridge2(motorParams.motorWiring ? 0u : 1u); 
	}
	return elecAngleStep;
}

