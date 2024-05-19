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
 
#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "stm32f10x.h"

#include "sine.h"


#define VOLT_DIV_RATIO(R1, R2) (((float) R2 / ((float)R1 + (float)R2)))
#define ADC_12bit 4096
//MCU power supply
#define mVREFINT 1200u


//SW
#define	PIN_SW              GPIOC
#define	PIN_F1_KEY 			GPIO_Pin_14
#define	PIN_F1_INT 			EXTI_Line14
#define	PIN_F2_KEY 			GPIO_Pin_15
#define	PIN_F2_INT 			EXTI_Line15
#define	PIN_SW3_ENTER		PIN_F1_KEY

#define	GPIO_JP             GPIOA
#define	PIN_JP1				GPIO_Pin_3
#define	PIN_JP2				GPIO_Pin_4
#define	PIN_JP3				GPIO_Pin_5


//A4950
#define PIN_A4950_VREF     	GPIOB
#define PIN_A4950_VREF12    GPIO_Pin_5	//TIM3_CH2
#define PIN_A4950_VREF34    GPIO_Pin_4	//TIM3_CH1

#define RS_A4950            100U //mOhm

#define PIN_A4950     		GPIOA
#define PIN_A4950_IN1   	GPIO_Pin_8
#define PIN_A4950_IN2   	GPIO_Pin_9
#define PIN_A4950_IN3  		GPIO_Pin_10
#define PIN_A4950_IN4  		GPIO_Pin_11

#define PIN_A4950_ENABLE    GPIO_Pin_6
    
#define	VREF_TIM			TIM3
#define VREF_TIM_MAX		(SINE_MAX>>VREF_SCALER)  //timer threshold - higher frequency timer works better with voltage low pass filter - less ripple

#define PWM_TIM             TIM1
#define PWM_TIM_MAX         (SINE_MAX>>PWM_SCALER)
#define PWM_TIM_MIN         1

#define GPIO_LSS            GPIOA
#define PIN_LSS_A           GPIO_Pin_0
#define PIN_LSS_B           GPIO_Pin_1
#define ADC_CH_LSS_A        ADC_Channel_0
#define ADC_CH_LSS_B        ADC_Channel_1
#define ADC_LSS             ADC1
#define LSS_OP_OFFSET       0.080f //V


//DClink
//DClink v_mot adc
#define GPIO_VMOT           GPIOB
#define PIN_VMOT            GPIO_Pin_1
#define ADC_CH_VMOT         ADC_Channel_9
#define ADC_VMOT            ADC1
#define R1_VDIV_VMOT        56.0 //kohm
#define R2_VDIV_VMOT        5.6  //kohm

//DClink v_bat adc
#define GPIO_VBAT           GPIOA
#define PIN_VBAT            GPIO_Pin_7
#define ADC_CH_VBAT         ADC_Channel_7
#define ADC_VBAT            ADC1
#define R1_VDIV_VBAT        56.0 //kohm
#define R2_VDIV_VBAT        5.6  //kohm


//A1333
#define PIN_A1333     		GPIOB
#define PIN_A1333_CS    	GPIO_Pin_12
#define PIN_A1333_SCK   	GPIO_Pin_13
#define PIN_A1333_MISO   	GPIO_Pin_14
#define PIN_A1333_MOSI   	GPIO_Pin_15

//TLE5012B
#define TLE5012B_SPIx		2
#define PIN_TLE5012B		GPIOB
#define PIN_TLE5012B_CS		GPIO_Pin_12
#define PIN_TLE5012B_SCK	GPIO_Pin_13
#define PIN_AUX_3_3			GPIO_Pin_14
#define PIN_TLE5012B_DATA   GPIO_Pin_15
#if (TLE5012B_SPIx == 1)
    #define TLE5012B_SPI	SPI1
#endif
#if (TLE5012B_SPIx == 2)
    #define TLE5012B_SPI	SPI2
#endif
    

//LED
#define GPIO_LED_RED		GPIOB
#define	PIN_LED_RED			GPIO_Pin_0
#define GPIO_LED_BLUE		GPIOC
#define PIN_LED_BLUE		GPIO_Pin_13

void board_init(void);

bool F1_button_state(void);
bool F2_button_state(void);

void Set_Error_LED(bool state);
void Set_Func_LED(bool state);

void adc_update_all(void);

float GetVDDA(void);
uint16_t GetMcuVoltage_mV(void);
float GetChipTemp(void);
float GetMotorVoltage(void); //V_mot
uint16_t GetMotorVoltage_mV(void);
float GetSupplyVoltage(void); //V_bat
uint16_t GetSupplyVoltage_mV(void);
float Get_PhaseA_Current(void);
float Get_PhaseB_Current(void);

#define MHz_to_Hz	(uint32_t)(1000000)
#define s_to_ms 	(1000U)
#define s_to_us 	(1000000U)
#define us_to_ns 	(1000U)
void Motion_task_init(uint16_t taskPeriod);
void Serivice_task_init(void);

extern volatile bool motion_task_isr_enabled;
void Motion_task_enable(void);
void Motion_task_disable(void);

extern volatile bool motion_task_overrun;
extern volatile uint32_t motion_task_overrun_count;
extern volatile uint16_t motion_task_execution_us;

extern volatile bool service_task_overrun;
extern volatile uint32_t service_task_overrun_count;
extern volatile uint16_t service_task_execution_us;

#endif
