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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#ifndef __BOARD_H
#define __BOARD_H

#include "stm32f10x.h"
#include "A4950.h"


#define ADC_12bit 4096
//MCU power supply
#define mV_REF 3300u
#define V_REF (float)mV_REF/1000.0f

//OLED
#define PIN_OLED						GPIOB
#define PIN_OLED_CS					GPIO_Pin_12
#define PIN_OLED_D0					GPIO_Pin_15
#define PIN_OLED_DC					GPIO_Pin_13
#define PIN_OLED_D1    			    GPIO_Pin_14
#define PIN_OLED_R                      GPIOA
#define PIN_OLED_RST   			    GPIO_Pin_8


//SW
#define	PIN_SW                     GPIOC
#define	PIN_FCN_KEY 				GPIO_Pin_15
#define	PIN_SW3_ENTER				PIN_FCN_KEY

#define	GPIO_JP                     GPIOA
#define	PIN_JP1					GPIO_Pin_3
#define	PIN_JP2					GPIO_Pin_4
#define	PIN_JP3					GPIO_Pin_5


//A4950
#define PIN_A4950     			GPIOB
#define PIN_A4950_VREF12    GPIO_Pin_5	//TIM3_CH2
#define PIN_A4950_VREF34    GPIO_Pin_4	//TIM3_CH1

#define RS_A4950            (uint16_t) 100 //mOhm

#define PIN_A4950_IN1   	GPIO_Pin_6
#define PIN_A4950_IN2   	GPIO_Pin_7
#define PIN_A4950_IN3  		GPIO_Pin_8
#define PIN_A4950_IN4  		GPIO_Pin_9
    
#define	VREF_TIM			TIM3
#define VREF_MAX			(SINE_MAX>>VREF_SCALER)  //timer threshold - higher frequency timer works better with voltage low pass filter - less ripple

//A1333
#define PIN_A1333     			GPIOB
#define PIN_A1333_CS    		GPIO_Pin_12
#define PIN_A1333_SCK   		GPIO_Pin_13
#define PIN_A1333_MISO   		GPIO_Pin_14
#define PIN_A1333_MOSI   		GPIO_Pin_15

//TLE5012B
#ifdef ServoCAN
#define TLE5012B_SPI					SPI2
#define TLE5012B_SPI_Periph RCC_APB1Periph_SPI2
#define PIN_TLE5012B					GPIOB
#define PIN_TLE5012B_CS					GPIO_Pin_12
#define PIN_TLE5012B_SCK				GPIO_Pin_13
#define PIN_TLE5012B_DATA    			GPIO_Pin_15
#else
#define TLE5012B_SPI					SPI1
#define TLE5012B_SPI_Periph RCC_APB2Periph_SPI1
#define PIN_TLE5012B					GPIOA
#define PIN_TLE5012B_CS					GPIO_Pin_4
#define PIN_TLE5012B_SCK				GPIO_Pin_5
#define PIN_TLE5012B_DATA    			GPIO_Pin_7
#endif

//LED
#define GPIO_LED_RED				GPIOC
#define	PIN_LED_RED					GPIO_Pin_13
#define GPIO_LED_BLUE				GPIOB
#define PIN_LED_BLUE				GPIO_Pin_0

void board_init(void);

bool Fcn_button_state(void);

void Set_Error_LED(bool state);
void Set_Func_LED(bool state);
volatile float chip_temp;
float GetChipTemp(void);

#define MHz_to_Hz	(uint32_t)(1000000)
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
