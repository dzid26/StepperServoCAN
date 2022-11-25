 /* StepperServoCAN
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

#include "board.h"
#include "main.h"
#include "stm32f10x.h"
#include "MKS.h"
#include "can.h"
#include "A4950.h"
#include "sine.h"

//Init clock
static void CLOCK_init(void)
{	
	//SystemInit() with clocks settings is run from startup_stm32f103xb.S
	SystemCoreClockUpdate();

	//ADC clock
	RCC->CFGR |= (RCC_CFGR_ADCPRE & RCC_CFGR_ADCPRE_DIV6);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

//Init NVIC
static void NVIC_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	NVIC_SetPriority(SysTick_IRQn,15); //Not used currently
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; //��ռ���ȼ�Ϊ1(����ѭ��)
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //10ms loop
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_Init(&NVIC_InitStructure);
}

//Init TLE5012B				    
static void TLE5012B_init(void)
{
	if (TLE5012B_SPI == SPI2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	}else if (TLE5012B_SPI == SPI1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	}else{
		//Unsupported
	}
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = PIN_TLE5012B_SCK | PIN_TLE5012B_DATA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //SPI alternate function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(PIN_TLE5012B, &GPIO_InitStructure);
	
  	GPIO_InitStructure.GPIO_Pin = PIN_TLE5012B_CS;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //software NSS gpio switching - non-alternate function
 	GPIO_Init(PIN_TLE5012B, &GPIO_InitStructure);
	GPIO_SetBits(PIN_TLE5012B, PIN_TLE5012B_CS);//CS high - deselect device for now
	
	SPI_InitTypeDef SPI_InitStructure;	
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //4.5Mbit/s
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	 //X8+X4+X3+X2+1  J1850 - sadly to use hardware CRC8, SPI_DataSize would need to be change to 8bit - not ideal
	SPI_InitStructure.SPI_CRCPolynomial = 0x1D;
	SPI_Init (TLE5012B_SPI, &SPI_InitStructure);

}

//Init switch IO
static void SWITCH_init(void)
{
	//dip switches
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_FCN_KEY;
    GPIO_Init(PIN_SW, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_JP1 | PIN_JP2 | PIN_JP3;
	GPIO_Init(GPIO_JP, &GPIO_InitStructure);

}

//Init A4950
#define VREF_MAX			(SINE_MAX>>VREF_SCALER)  //timer threshold - higher frequency timer works better with voltage low pass filter - less ripple
static void A4950_init(void)
{	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//A4950 Input pins
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_A4950_IN1|PIN_A4950_IN2|PIN_A4950_IN3|PIN_A4950_IN4;
    GPIO_Init(PIN_A4950, &GPIO_InitStructure);

	//A4950 Vref pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //has to be enabled before remap
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//Release pins for VREF34, DIP2, DIP1
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = PIN_A4950_VREF12|PIN_A4950_VREF34;
    GPIO_Init(PIN_A4950, &GPIO_InitStructure);

	//Remap to the upper pins

	//Init TIM3
	TIM_TimeBaseInitTypeDef  		TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = VREF_MAX;									
	TIM_TimeBaseStructure.TIM_Prescaler = 0;						//No prescaling - max speed 72MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(VREF_TIM, &TIM_TimeBaseStructure);
	
	
	TIM_OCInitTypeDef  	        TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(VREF_TIM, &TIM_OCInitStructure);	//CH1
	TIM_OC2Init(VREF_TIM, &TIM_OCInitStructure);	//CH2
	TIM_OC1PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
 
	TIM_Cmd(VREF_TIM, ENABLE);
}

//Init A1333
static void A1333_init (void)
{	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	//todo replace with std_periph functions:
	GPIOB->CRH &= 0x0000ffff;	//clean CS SCK MOSI MISO control bit
	GPIOB->CRH |= 0xb8b30000;	//config CS Universal push-pull output��SCK MOSI Multiplexed push-pull output��MISO pulldown input
	GPIOB->ODR |= 0x0000f000;	//default CS SCK MOSI MISO output high

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;													//CPOL=1
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;												//CPHA=1
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0x9; //X4+X+1
	SPI_Init (SPI2,&SPI_InitStructure);	
	SPI_Cmd (SPI2,ENABLE);

	// SPI2->CR1 |= SPI_CR1_CRCEN;		//CRC enable - to be tested. Probably DMA would be also beneficial
}

static void LED_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_LED_RED;
    GPIO_Init(GPIO_LED_RED, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PIN_LED_BLUE;
    GPIO_Init(GPIO_LED_BLUE, &GPIO_InitStructure);
}
static void CAN_begin(){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//CAN

	/* Configure CAN RX pin */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN TX pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN RS pin */
	// GPIO_Mode_IN_FLOATING for slew rate control
	// GPIO_Mode_Out_PP for highest speed or standby mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	CAN_InitTypeDef        CAN_InitStructure;
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;


	/* Baudrate = 500kbps*/
	if (SystemCoreClock == 72000000){
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
		CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;
		CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
		CAN_InitStructure.CAN_Prescaler=12;
	}
	if (SystemCoreClock == 64000000){
		CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
		CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;
		CAN_InitStructure.CAN_BS2=CAN_BS2_4tq;
		CAN_InitStructure.CAN_Prescaler=8;
	}
	
	CAN_Init(CAN1, &CAN_InitStructure);

	//setup filters
	CAN_MsgsFiltersSetup();
	//enable receive interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE); 
	CAN_ITConfig(CAN1, CAN_IT_FOV0, ENABLE); 

}


static void Analog_init(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

	ADC_DeInit(ADC1);
	ADC_InitTypeDef ADC_InitStructure;
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			  
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;		   
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_DiscModeCmd(ADC1, ENABLE);
	ADC_DiscModeChannelCountConfig(ADC1, 1);

	/* Enable the temperature sensor and vref internal channel */ 
	ADC_TempSensorVrefintCmd(ENABLE);    
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibaration register */ 
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));  
	/* Start ADC1 Software Conversion */ 

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = PIN_VMOT;
	GPIO_Init(GPIO_VMOT, &GPIO_InitStructure);

	/* ADC1 regular channe16 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC_VMOT, ADC_CH_VMOT, 2, ADC_SampleTime_13Cycles5);
}

static volatile float chip_temp_adc;
void ChipTemp_adc_update(){
	// ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_239Cycles5);  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)!=SET){
		//wait until conversion is finished
	}
	uint16_t adc_raw = ADC_GetConversionValue(ADC1);
	
	float adc_volt = ((float)adc_raw+0.5f) * V_REF / (float)ADC_12bit;
	
	const float t0 = 35.0f;
	const float adcVoltRef = 1.325f; //! calibrate at some t0
	const float tempSlop = 4.3f/1000; //typically 4.3mV per C
	chip_temp_adc = (adcVoltRef - adc_volt) / tempSlop + t0;
}

float GetChipTemp(){
	return chip_temp_adc;
}

static volatile float vmot_adc;
void Vmot_adc_update(){
	// ADC_RegularChannelConfig(ADC_VMOT, ADC_CH_VMOT, 1, ADC_SampleTime_13Cycles5);  
	ADC_SoftwareStartConvCmd(ADC_VMOT, ENABLE);
	while (ADC_GetFlagStatus(ADC_VMOT, ADC_FLAG_EOC)!=SET){
	//wait until conversion is finished
	}
	uint16_t adc_raw = ADC_GetConversionValue(ADC_VMOT);

	float adc_volt = ((float)adc_raw+0.5f) * V_REF / (float)ADC_12bit;
	vmot_adc = adc_volt / VOLT_DIV_RATIO(R1_VDIV_VMOT, R2_VDIV_VMOT);
}

float GetMotorVoltage(){
	return vmot_adc;
}

void board_init(void)
{
	CLOCK_init();
	NVIC_init(); 
	A4950_init();
	Analog_init();
	TLE5012B_init();
	SWITCH_init();
	LED_init();
	CAN_begin();
}

bool Fcn_button_state(void)
{
	if (GPIO_ReadInputDataBit(PIN_SW, PIN_FCN_KEY) == Bit_RESET){
		return true; //low is pressed
	}
	return false;
}

//Set LED state
//true - on, false - off
void Set_Func_LED(bool state)
{
	GPIO_WriteBit(GPIO_LED_BLUE, PIN_LED_BLUE, (BitAction)(state));
}

//Set LED state
//true - on, false - off
void Set_Error_LED(bool state)
{
	GPIO_WriteBit(GPIO_LED_RED, PIN_LED_RED, (BitAction)(state));
}

#define MOTION_TASK_TIM TIM1
#define SERVICE_TASK_TIM  TIM2

void Motion_task_init(uint16_t taskPeriod)
{
	//setup timer
	TIM_DeInit(MOTION_TASK_TIM);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock / MHz_to_Hz - 1U; //Prescale to 1MHz - 1uS
	TIM_TimeBaseStructure.TIM_Period = taskPeriod - 1U;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //has to be zero to not skip period ticks
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(MOTION_TASK_TIM, &TIM_TimeBaseStructure);

	TIM_SetCounter(MOTION_TASK_TIM, 0);
	TIM_Cmd(MOTION_TASK_TIM, ENABLE);
}


void Serivice_task_init(void){
	//setup timer
	TIM_DeInit(SERVICE_TASK_TIM);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	

	//Init SERVICE_TASK_TIM
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 	//cppcheck-suppress  naming-varname - HAL library
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / MHz_to_Hz - 1);	//Prescale timer clock to 1MHz - 1us period
	TIM_TimeBaseStructure.TIM_Period = (10 * 1000) - 1;	//10ms = 10000us
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //has to be zero to not skip period ticks
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(SERVICE_TASK_TIM, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(SERVICE_TASK_TIM, TIM_IT_Update);
	TIM_ClearFlag(SERVICE_TASK_TIM, TIM_FLAG_Update);
	TIM_ITConfig(SERVICE_TASK_TIM, TIM_IT_Update,ENABLE);

	TIM_SetCounter(SERVICE_TASK_TIM, 0);
	TIM_Cmd(SERVICE_TASK_TIM, ENABLE);
}

void TIM1_UP_IRQHandler(void);
void TIM2_IRQHandler(void);

volatile bool motion_task_isr_enabled = false;

//enable motor fast loop interrupt
void Motion_task_enable(void)
{
	motion_task_isr_enabled = true;
	TIM_ClearITPendingBit(MOTION_TASK_TIM, TIM_IT_Update);
	TIM_ITConfig(MOTION_TASK_TIM, TIM_IT_Update, ENABLE);
}

//disable motor fast loop interrupt
void Motion_task_disable(void)
{
	motion_task_isr_enabled = false;
	TIM_ITConfig(MOTION_TASK_TIM, TIM_IT_Update, DISABLE);
	TIM_ClearITPendingBit(MOTION_TASK_TIM, TIM_IT_Update);
}

volatile bool motion_task_overrun;
volatile uint32_t motion_task_overrun_count;
volatile uint16_t motion_task_execution_us;

volatile bool service_task_overrun;
volatile uint32_t service_task_overrun_count;
volatile uint16_t service_task_execution_us;

void TIM1_UP_IRQHandler(void) //precise fast independant timer for motor control
{
	if(TIM_GetITStatus(MOTION_TASK_TIM, TIM_IT_Update) != RESET)
	{	
		TIM_ClearITPendingBit(MOTION_TASK_TIM, TIM_IT_Update);

		// ! Call the task here !
		Motion_task();
		
		//Task diagnostic
		motion_task_execution_us = TIM_GetCounter(MOTION_TASK_TIM); //get current timer value in uS thanks to the prescaler
		if(TIM_GetITStatus(MOTION_TASK_TIM, TIM_IT_Update) != RESET) //if timer reset during execution, we have an overrun
		{
			motion_task_overrun = true;
			motion_task_execution_us += MOTION_TASK_TIM->ARR; //assume that timer rolled over and add the full period to the current value
			motion_task_overrun_count++;
			TIM_ClearITPendingBit(MOTION_TASK_TIM, TIM_IT_Update); //don't allow to reenter this task if it overruns. Being highest priority it would block everything.
		}else{
			motion_task_overrun = false;
		}
		Set_Error_LED(motion_task_overrun); //show the error LED
	}
}

void TIM2_IRQHandler(void) //TIM2 used for task triggering
{
	if(TIM_GetITStatus(SERVICE_TASK_TIM, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(SERVICE_TASK_TIM, TIM_IT_Update);
		
		// ! Call the task here !
		Service_task();
		
		//Task diagnostic
		service_task_execution_us = TIM_GetCounter(SERVICE_TASK_TIM); //get current timer value in uS thanks to the prescaler
		if(TIM_GetITStatus(SERVICE_TASK_TIM, TIM_IT_Update) != RESET) //if timer reset during execution, we have an overrun
		{
			service_task_overrun = true;
			service_task_execution_us += SERVICE_TASK_TIM->ARR; //assume that timer rolled over and add the full period to the current value
			service_task_overrun_count++;
		}else
		{
			service_task_overrun = false;
		}

	}
}
