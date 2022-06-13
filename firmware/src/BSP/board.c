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

#include "board.h"

//Init clock
static void CLOCK_init(void)
{	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 	//CAN, INPUTS, SWITCH, TLE5012B_SPI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//VREF12, VREF34, OLED S42B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//LED

}

//Init NVIC
static void NVIC_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //��4��:4λ��ռ���ȼ�
	NVIC_SetPriority(SysTick_IRQn,15); //����SysTick_IRQn��ռ���ȼ����
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; //��ռ���ȼ�0(dir����)
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; //��ռ���ȼ�Ϊ1(����ѭ��)
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
}

//Init IO
static void INPUT_init(void)
{	
	GPIOA->CRL &= 0xfffff000;	//clean STEP(PA0) DIR(PA1) ENABLE(PA2) control bit
	GPIOA->CRL |= 0x00000888;	//config STEP(PA0) DIR(PA1) ENABLE(PA2) input mode	
	GPIOA->ODR |= 0x00000007;	//default STEP(PA0) DIR(PA1) ENABLE(PA2) pullup
}

//Init TLE5012B				    
static void TLE5012B_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = PIN_TLE5012B_SCK | GPIO_Pin_6 | PIN_TLE5012B_DATA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/14/15 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
	SPI_InitTypeDef SPI_InitStructure;	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 0x8E; //X8+X4+X3+X2+1  J1850
	SPI_Init (SPI1,&SPI_InitStructure);	
	SPI_Cmd (SPI1,ENABLE);

	
  	GPIO_InitStructure.GPIO_Pin = PIN_TLE5012B_CS;				 //PA4  
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(PIN_TLE5012B, &GPIO_InitStructure);
		
  	GPIO_SetBits(PIN_TLE5012B, PIN_TLE5012B_CS);
}


//Init SSD1306				    
static void OLED_init(void)
{	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_OLED_CS|PIN_OLED_D0|PIN_OLED_DC|PIN_OLED_D1;
    GPIO_Init(PIN_OLED, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PIN_OLED_RST;
	GPIO_Init(PIN_OLED_R, &GPIO_InitStructure);


}

//Init switch IO
static void SWITCH_init(void)
{
	#ifdef MKS
	GPIOA->CRH &= 0xfffff000;	//SW4 SW3 SW1 (PA8 PA9 PA10)	
	GPIOA->CRH |= 0x00000888;	//SW4 SW3 SW1 (PA8 PA9 PA10)	
	GPIOA->ODR |= 0x00000700;	//SW4 SW3 SW1 (PA8 PA9 PA10)
	#elif BTT

	//dip switches
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_DIP2|PIN_DIP3|PIN_DIP4|PIN_SW4_MENU|PIN_SW3_ENTER;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PIN_DIP1 | PIN_SW1_NEXT|PIN_SW4_EXIT;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	#endif
}

//Init A4950
static void A4950_init(void)
{	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

#ifdef MKS
	GPIOB->CRL &= 0x00ffff00;	//clean VREF12(PB0) VREF34(PB1) IN1(PB6) IN2(PB7) control bit
	GPIOB->CRL |= 0x330000bb;	//config VREF12(PB0) VREF34(PB1)Multiplexed push-pull output IN1(PB6) IN2(PB7) Universal push-pull output
	GPIOB->CRH &= 0xffffff00;	//clean IN3(PB8) IN4(PB9) control bit	
	GPIOB->CRH |= 0x00000033;	//config IN3(PB8) IN4(PB9) Universal push-pull output	
	GPIOB->ODR &= 0xfffffc3f;	//defualt IN1(PB6) IN2(PB7) IN3(PB8) IN4(PB9) 
#elif BTT
	//A4950 Input pins
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
#endif

	//Init TIM3
	TIM_TimeBaseInitTypeDef  		TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = VREF_MAX;									//reload c
	TIM_TimeBaseStructure.TIM_Prescaler = 0;											//72MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(VREF_TIM, &TIM_TimeBaseStructure);
	
	
	TIM_OCInitTypeDef  	        TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
#ifdef MKS
	TIM_OC3Init(VREF_TIM, &TIM_OCInitStructure);	//TIM3 CH3
	TIM_OC4Init(VREF_TIM, &TIM_OCInitStructure);	//TIM3 CH4
	TIM_OC3PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
#elif BTT
	TIM_OC1Init(VREF_TIM, &TIM_OCInitStructure);	//TIM3 CH1
	TIM_OC2Init(VREF_TIM, &TIM_OCInitStructure);	//TIM3 CH2
	TIM_OC1PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
 #endif
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
#ifdef MKS
	//POWER_LED
	GPIOA->CRL &= 0xffff0fff; //PA3
	GPIOA->CRL |= 0x00003000; //Universal push-pull output
	GPIOA->ODR |= 0x00000008;
	
#elif BTT
	GPIO_InitTypeDef  GPIO_InitStructure; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = PIN_LED_WORK;
    GPIO_Init(PIN_LED, &GPIO_InitStructure); 
#endif
}
static void CAN_begin(){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//CAN

	/* Configure CAN pin: RX */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
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
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler=12;
	CAN_Init(CAN1, &CAN_InitStructure);

	//setup filters
	CAN_MsgsFiltersSetup();
	//enable receive interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE); 
	CAN_ITConfig(CAN1, CAN_IT_FOV0, ENABLE); 

}


static void ChipTemp_init(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

	ADC_InitTypeDef ADC_InitStructure;
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			  
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;		   
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channe16 configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);  
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

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)!=SET);

}


void board_init(void)
{
	CLOCK_init();
	NVIC_init(); 
	INPUT_init();
	A4950_init();
#ifdef MKS
	A1333_init();
	ChipTemp_init();
#elif BTT
	TLE5012B_init();
#endif
	OLED_init();
	SWITCH_init();
	LED_init();
	CAN_begin();
}

//red led
//state = true  light up
//state = false dim
void POWER_LED(bool state)
{
	#ifdef MKS
	GPIO_WriteBit(PIN_RED, PIN_LED_RED, (BitAction)state);
	#endif
}

//blue led
//state(error) = light up
//state(error) = dim
void WORK_LED(bool state)
{
	GPIO_WriteBit(PIN_LED, PIN_LED_WORK, (BitAction)(state));
}


void setupTCInterrupts(uint16_t period)
{
	//setup timer
	TIM_DeInit(TIM1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_ClocksTypeDef clocksData;
	RCC_GetClocksFreq(&clocksData);

	TIM_TimeBaseStructure.TIM_Prescaler = (clocksData.PCLK2_Frequency / MHz_to_Hz - 1);	//Prescale to 1MHz - Timer 1 is clocked by PCLK2 (abp2)
	TIM_TimeBaseStructure.TIM_Period = period-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //has to be zero to not skip period ticks
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_SetCounter(TIM1, 0);
	TIM_Cmd(TIM1, ENABLE);
}

volatile bool TC1_ISR_Enabled = false;
void enableTCInterrupts(void)
{
	TC1_ISR_Enabled = true;
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

void enableTCInterruptsCond(bool previously_enabled)
{
	if(previously_enabled){
		enableTCInterrupts();
	}
}

void disableTCInterrupts(void)
{
	TC1_ISR_Enabled = false;
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}
