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
 * along with this program.  If not, see <www.gnu.org/licenses/>.
 *
 */

#include "board.h"
#include "main.h"
#include "can.h"
#include "A4950.h"
#include "stepper_controller.h"
#include "utils.h"

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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

//Init NVIC
static void NVIC_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	NVIC_InitTypeDef nvic_initStructure;
	nvic_initStructure.NVIC_IRQChannelCmd = ENABLE;
	
	nvic_initStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;//PWM_TIM break-in
	nvic_initStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_initStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_initStructure);

	nvic_initStructure.NVIC_IRQChannel = TIM4_IRQn;//MOTION_TASK_TIM
	nvic_initStructure.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_initStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_initStructure);	

	nvic_initStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn; //CAN bus
	nvic_initStructure.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_initStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_initStructure);

	nvic_initStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //F1, F2 keys
	nvic_initStructure.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_initStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic_initStructure);

	nvic_initStructure.NVIC_IRQChannel = TIM2_IRQn; //SERVICE_TASK_TIM
	nvic_initStructure.NVIC_IRQChannelPreemptionPriority = 4;
	nvic_initStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_initStructure);
}

//Init TLE5012B				    
static void TLE5012B_init(void)
{
	#if (TLE5012B_SPIx == 1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	#endif
	#if (TLE5012B_SPIx == 2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	#endif

	GPIO_InitTypeDef  gpio_initStructure;
	gpio_initStructure.GPIO_Pin = PIN_TLE5012B_SCK | PIN_TLE5012B_DATA;
	gpio_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //SPI alternate function
	gpio_initStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(PIN_TLE5012B, &gpio_initStructure);
	
  	gpio_initStructure.GPIO_Pin = PIN_TLE5012B_CS;
 	gpio_initStructure.GPIO_Mode = GPIO_Mode_Out_PP; //software NSS gpio switching - non-alternate function
 	GPIO_Init(PIN_TLE5012B, &gpio_initStructure);
	GPIO_SetBits(PIN_TLE5012B, PIN_TLE5012B_CS);//CS high - deselect device for now

	SPI_InitTypeDef spi_initStructure;	
	spi_initStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	spi_initStructure.SPI_Mode = SPI_Mode_Master;
	spi_initStructure.SPI_DataSize = SPI_DataSize_16b;
	spi_initStructure.SPI_CPOL = SPI_CPOL_Low;
	spi_initStructure.SPI_CPHA = SPI_CPHA_2Edge;
	spi_initStructure.SPI_NSS = SPI_NSS_Soft;
	#if (TLE5012B_SPIx == 1)
		spi_initStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8Mbps at 64Mhz PLCK2
	#endif
	#if (TLE5012B_SPIx == 2)
		spi_initStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //8Mbps at 32Mhz PLCK1
	#endif
	
	spi_initStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	 //X8+X4+X3+X2+1  J1850 - sadly to use hardware CRC8, SPI_DataSize would need to be change to 8bit - not ideal
	spi_initStructure.SPI_CRCPolynomial = 0x1D;
	SPI_Init(TLE5012B_SPI, &spi_initStructure);

}

//Init switch IO
static void SWITCH_init(void)
{
	//switches
	GPIO_InitTypeDef  gpio_initStructure; 
	gpio_initStructure.GPIO_Mode = GPIO_Mode_IPU;
 	gpio_initStructure.GPIO_Speed = GPIO_Speed_2MHz;
    gpio_initStructure.GPIO_Pin = PIN_F1_KEY | PIN_F2_KEY;
    GPIO_Init(PIN_SW, &gpio_initStructure);

	EXTI_DeInit();
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource15);
	EXTI_InitTypeDef EXTIInitStruct;
	EXTIInitStruct.EXTI_Line = PIN_F1_INT | PIN_F2_INT;
	EXTIInitStruct.EXTI_LineCmd = ENABLE;
	EXTIInitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTIInitStruct);

	gpio_initStructure.GPIO_Pin = PIN_JP1 | PIN_JP2;
	GPIO_Init(GPIO_JP, &gpio_initStructure);

  	gpio_initStructure.GPIO_Pin = PIN_AUX_3_3;
 	gpio_initStructure.GPIO_Mode = GPIO_Mode_IPU; //aux 3.3 mosfet - On by default
 	GPIO_Init(GPIOB, &gpio_initStructure);

}

//Init A4950
static void A4950_init(void)
{	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //VREF_TIM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //PWM_TIM

	GPIO_InitTypeDef  		gpio_initStructure;
	TIM_TimeBaseInitTypeDef timeBaseStructure;
	TIM_OCInitTypeDef		tim_OCInitStructure;


	//A4950 Vref pins
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//Release pins for VREF34, DIP2, DIP1
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);	 	//VREF_TIM

	gpio_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 	gpio_initStructure.GPIO_Speed = GPIO_Speed_2MHz;
	gpio_initStructure.GPIO_Pin = PIN_A4950_VREF12|PIN_A4950_VREF34;
    GPIO_Init(PIN_A4950_VREF, &gpio_initStructure);

	//Init VREF_TIM
	TIM_DeInit(VREF_TIM);
	timeBaseStructure.TIM_Period = VREF_TIM_MAX;
	timeBaseStructure.TIM_Prescaler = 0;						//No prescaling - max speed 72MHz
	timeBaseStructure.TIM_ClockDivision = 0;
	timeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(VREF_TIM, &timeBaseStructure);
	
	
	tim_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	tim_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	tim_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	tim_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(VREF_TIM, &tim_OCInitStructure);	//CH1
	TIM_OC2Init(VREF_TIM, &tim_OCInitStructure);	//CH2
	TIM_OC1PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(VREF_TIM, TIM_OCPreload_Enable);
 
	TIM_Cmd(VREF_TIM, ENABLE);

	//A4950 Input pins
	gpio_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
 	gpio_initStructure.GPIO_Speed = GPIO_Speed_2MHz;
    gpio_initStructure.GPIO_Pin = A4950_A_IN1|A4950_A_IN2|A4950_B_IN1|A4950_B_IN2;
    GPIO_Init(PIN_A4950, &gpio_initStructure);
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);//PWM_TIM
	
	TIM_DeInit(PWM_TIM);
	//Init PWM_TIM - A4950_A_IN1|A4950_A_IN2|A4950_B_IN1|A4950_B_IN2
	timeBaseStructure.TIM_Period = PWM_TIM_MAX;
	timeBaseStructure.TIM_Prescaler = 0;	//No prescaling - max cpu speed (MHz)
	timeBaseStructure.TIM_ClockDivision = 0;
	timeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseInit(PWM_TIM, &timeBaseStructure);

	tim_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	tim_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	tim_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
	tim_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	tim_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;   // this is necessary even though OCN pins are not used
	tim_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; //Motor coasting when idle is set to reset
	tim_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	tim_OCInitStructure.TIM_Pulse = 0;

	TIM_OC1Init(PWM_TIM, &tim_OCInitStructure);	//CH1
	TIM_OC2Init(PWM_TIM, &tim_OCInitStructure);	//CH2

	// invert second phase activation to avoid current draw in the same time
	tim_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	tim_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  // this is necessary even though OCN pins are not used
	TIM_OC3Init(PWM_TIM, &tim_OCInitStructure);	//CH3
	TIM_OC4Init(PWM_TIM, &tim_OCInitStructure);	//CH4

	TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(PWM_TIM, TIM_OCPreload_Disable);

	// Configure PWM_TIM break
	gpio_initStructure.GPIO_Pin = PIN_A4950_ENABLE;
	gpio_initStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio_initStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PIN_A4950, &gpio_initStructure);

	TIM_BDTRInitTypeDef tim_BDTRInitStructure = {0};
	tim_BDTRInitStructure.TIM_Break = TIM_Break_Disable;  // If enabled, keep EN port low to activate motor
	tim_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	tim_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_3;
	TIM_BDTRConfig(PWM_TIM, &tim_BDTRInitStructure);
	TIM_ITConfig(PWM_TIM, TIM_IT_Break, ENABLE);

	TIM_Cmd(PWM_TIM, ENABLE);
}

void TIM1_BRK_IRQHandler(void){ //PWM_TIM break-in
	if(TIM_GetITStatus(PWM_TIM, TIM_IT_Break) != RESET) {
		TIM_ClearITPendingBit(PWM_TIM, TIM_IT_Break);
		// Break stop will not be smooth, but request soft off anyway in order not to be able to enable without requesting full off first
		StepperCtrl_setMotionMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
	}
}

static void LED_init(void)
{
	GPIO_InitTypeDef  gpio_initStructure; 
	gpio_initStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 	gpio_initStructure.GPIO_Speed = GPIO_Speed_2MHz;
    gpio_initStructure.GPIO_Pin = PIN_LED_RED;
    GPIO_Init(GPIO_LED_RED, &gpio_initStructure);
    gpio_initStructure.GPIO_Pin = PIN_LED_BLUE;
    GPIO_Init(GPIO_LED_BLUE, &gpio_initStructure);
}
static void CAN_begin(void){

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//CAN
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);	//CAN1 remap to other pins

	/* Configure CAN RX pin */
	GPIO_InitTypeDef gpio_initStructure;
	gpio_initStructure.GPIO_Pin = GPIO_Pin_8;
	gpio_initStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio_initStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &gpio_initStructure);

	/* Configure CAN TX pin */
	gpio_initStructure.GPIO_Pin = GPIO_Pin_9;
	gpio_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_initStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &gpio_initStructure);

	/* Configure CAN RS pin */
	// GPIO_Mode_Out_PP low for max speed at short distances (20V/us)
	// GPIO_Mode_IPD for slew rate control (5V/us)
	// GPIO_Mode_Out_PP high for standby mode
	gpio_initStructure.GPIO_Pin = GPIO_Pin_2;
	gpio_initStructure.GPIO_Mode = GPIO_Mode_IPD;
    gpio_initStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio_initStructure);
	
	CAN_InitTypeDef        can_initStructure;
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&can_initStructure);

	/* CAN cell init */
	can_initStructure.CAN_TTCM=DISABLE;
	can_initStructure.CAN_ABOM=ENABLE;
	can_initStructure.CAN_AWUM=DISABLE;
	can_initStructure.CAN_NART=DISABLE;
	can_initStructure.CAN_RFLM=DISABLE;
	can_initStructure.CAN_TXFP=DISABLE;
	can_initStructure.CAN_Mode=CAN_Mode_Normal;


	/* Baudrate = 500kbps*/
	if (SystemCoreClock == 72000000){
		can_initStructure.CAN_SJW=CAN_SJW_1tq;
		can_initStructure.CAN_BS1=CAN_BS1_2tq;
		can_initStructure.CAN_BS2=CAN_BS2_3tq;
		can_initStructure.CAN_Prescaler=12;
	}
	if (SystemCoreClock == 64000000){
		can_initStructure.CAN_SJW=CAN_SJW_1tq;
		can_initStructure.CAN_BS1=CAN_BS1_3tq;
		can_initStructure.CAN_BS2=CAN_BS2_4tq;
		can_initStructure.CAN_Prescaler=8;
	}
	
	CAN_Init(CAN1, &can_initStructure);

	//setup filters
	CAN_MsgsFiltersSetup();
	//enable receive interrupt
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN1, CAN_IT_FF0, ENABLE); 
	CAN_ITConfig(CAN1, CAN_IT_FOV0, ENABLE); 

}

static void Vrefint_adc_update(void);

static void Analog_init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //div6 default

	
	GPIO_InitTypeDef  gpio_initStructure;
	gpio_initStructure.GPIO_Mode = GPIO_Mode_AIN;
    gpio_initStructure.GPIO_Pin = PIN_VMOT;
	GPIO_Init(GPIO_VMOT, &gpio_initStructure);

	gpio_initStructure.GPIO_Pin = PIN_VBAT;
	GPIO_Init(GPIO_VBAT, &gpio_initStructure);
	
    gpio_initStructure.GPIO_Pin = PIN_LSS_A|PIN_LSS_B;
	GPIO_Init(GPIO_LSS, &gpio_initStructure);


	ADC_DeInit(ADC1);
	ADC_InitTypeDef adc_initStructure;
	/* ADC1 configuration ------------------------------------------------------*/
	adc_initStructure.ADC_Mode = ADC_Mode_Independent;	 
	adc_initStructure.ADC_ScanConvMode = DISABLE;			  
	adc_initStructure.ADC_ContinuousConvMode = DISABLE;	 
	adc_initStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	adc_initStructure.ADC_DataAlign = ADC_DataAlign_Right;		   
	adc_initStructure.ADC_NbrOfChannel = 5;
	ADC_Init(ADC1, &adc_initStructure);

	ADC_DiscModeCmd(ADC1, ENABLE);
	ADC_DiscModeChannelCountConfig(ADC1, 1);

	/* Enable the temperature sensor and vref internal channel */ 
	ADC_TempSensorVrefintCmd(ENABLE);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	/* Enable ADC1 reset calibaration register */ 
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1) == SET){
		//wait for adc calibration reset
	}
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1) == SET){
		//wait for adc calibration finish
	}

	//Measure VDDA shortly after ADC calibration
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_71Cycles5);
	Vrefint_adc_update();
	ADC_SoftwareStartConvCmd(ADC1, DISABLE);

	/* ADC1 regular configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC_VMOT, ADC_CH_VMOT, 2, ADC_SampleTime_13Cycles5);
	ADC_RegularChannelConfig(ADC_VBAT, ADC_CH_VBAT, 3, ADC_SampleTime_13Cycles5);
	ADC_RegularChannelConfig(ADC_LSS, ADC_CH_LSS_A, 4, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC_LSS, ADC_CH_LSS_B, 5, ADC_SampleTime_239Cycles5);
}

static uint16_t Get_ADC_raw_nextRank(ADC_TypeDef* adcx){
	ADC_SoftwareStartConvCmd(adcx, ENABLE);
	while (ADC_GetFlagStatus(adcx, ADC_FLAG_EOC)!=SET){
		//wait until conversion is finished
	}
	uint16_t adc_raw = ADC_GetConversionValue(adcx);
	return adc_raw;
}

//ADC raw to voltage
static uint16_t vrefint_adc = 1489U; //VDDA 3.3V @ 1.2V vrefint
static float covnert_ADC_raw_volt(uint16_t adc_raw){
	float adc_volt = (float)(uint16_t)((uint32_t)adc_raw * mVREFINT / vrefint_adc) / 1000U;
	return adc_volt;
}

static uint16_t  vdda_adc_mV;
//called only once during boot up
static void Vrefint_adc_update(void){
	vrefint_adc = Get_ADC_raw_nextRank(ADC1);
	assert(vrefint_adc > 1357); //VDDA above 3.5V @ 1.16V vrefint
	assert(vrefint_adc < 1587); //VDDA below 3.2V @ 1.24V vrefint
	vdda_adc_mV = (uint16_t)(float)(GetVDDA() * (float)V_TO_mV);
}

float GetVDDA(void){
	return covnert_ADC_raw_volt(ADC_12bit); //gets VDDA, 
}

uint16_t GetMcuVoltage_mV(void){
	return vdda_adc_mV;
}

static float chip_temp_adc;
static void ChipTemp_adc_update(void){
	float adc_volt = covnert_ADC_raw_volt(Get_ADC_raw_nextRank(ADC1));
	const float t0 = 25.0f; //deg Celsius
	const float adcVolt_t0 = 1.385f; //! calibrate at some t0
	const float tempSlop = 4.3f/1000.0f; //typically 4.3mV per C
	chip_temp_adc = ((adcVolt_t0 - adc_volt) / tempSlop) + t0;
}

float GetChipTemp(void){
	return chip_temp_adc;
}

static float vmot_adc;
static uint16_t vmot_adc_mV;
static void Vmot_adc_update(void){
	float adc_volt = covnert_ADC_raw_volt(Get_ADC_raw_nextRank(ADC_VMOT));
	vmot_adc = adc_volt / VOLT_DIV_RATIO(R1_VDIV_VMOT, R2_VDIV_VMOT);
	vmot_adc_mV = (uint16_t)(float)(vmot_adc * (float)V_TO_mV);
}

float GetMotorVoltage(void){
	return vmot_adc;
}
uint16_t GetMotorVoltage_mV(void){
	return vmot_adc_mV;
}

static float vbat_adc;
static uint16_t vbat_adc_mV;
static void Vbat_adc_update(void){
	float adc_volt = covnert_ADC_raw_volt(Get_ADC_raw_nextRank(ADC_VBAT));
	vbat_adc = adc_volt / VOLT_DIV_RATIO(R1_VDIV_VBAT, R2_VDIV_VBAT);
	vbat_adc_mV = (uint16_t)(float)(vbat_adc * (float)V_TO_mV);
}

float GetSupplyVoltage(void){
	return vbat_adc;
}
uint16_t GetSupplyVoltage_mV(void){
	return vbat_adc_mV;
}

static float lssA_adc;
static float lssB_adc;
static void LSS_adc_update(void){ //todo add filter
	float adc_volt;
	adc_volt = covnert_ADC_raw_volt(Get_ADC_raw_nextRank(ADC_LSS));
	lssA_adc = max(adc_volt-LSS_OP_OFFSET, 0.0f) / 9.2f * 10.0f;
	adc_volt = covnert_ADC_raw_volt(Get_ADC_raw_nextRank(ADC_LSS));
	lssB_adc = max(adc_volt-LSS_OP_OFFSET, 0.0f) / 9.2f * 10.0f;
}

float Get_PhaseA_Current(void){
	return lssA_adc;
}
float Get_PhaseB_Current(void){
	return lssB_adc;
}

void adc_update_all(void){
	//order matters - see ADC1 regular configuration above
	ChipTemp_adc_update();
	Vmot_adc_update();
	Vbat_adc_update();
	LSS_adc_update();
}

void board_init(void)
{
	CLOCK_init();
	A4950_init();
	Analog_init();
	TLE5012B_init();
	SWITCH_init();
	LED_init();
	CAN_begin();
	NVIC_init(); 
}

static bool F1_button;
static bool F2_button;
void EXTI15_10_IRQHandler(void){
	if (EXTI_GetITStatus(PIN_F1_INT) != RESET){
		if (GPIO_ReadInputDataBit(PIN_SW, PIN_F1_KEY) == Bit_RESET){
			F1_button = true; //low is pressed
		}else{
			F1_button = false; //high is released
		}
		EXTI_ClearITPendingBit(PIN_F1_INT);
	}
	if (EXTI_GetITStatus(PIN_F2_INT) != RESET){
		if (GPIO_ReadInputDataBit(PIN_SW, PIN_F2_KEY) == Bit_RESET){
			F2_button = true; //low is pressed
		}else{
			F2_button = false; //high is released
		}
		EXTI_ClearITPendingBit(PIN_F2_INT);
	}
}

bool F1_button_state(void){
	return F1_button;
}

bool F2_button_state(void){
	return F2_button;
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

#define MOTION_TASK_TIM TIM4
#define SERVICE_TASK_TIM  TIM2

void Motion_task_init(uint16_t taskPeriod)
{
	//setup timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//MOTION_TASK_TIM

    TIM_DeInit(MOTION_TASK_TIM);
	TIM_TimeBaseInitTypeDef timeBaseStructure;
	timeBaseStructure.TIM_Prescaler = SystemCoreClock / MHz_to_Hz - 1U; //Prescale to 1MHz - 1uS
	timeBaseStructure.TIM_Period = taskPeriod - 1U;
	timeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timeBaseStructure.TIM_RepetitionCounter = 0; //has to be zero to not skip period ticks
	timeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(MOTION_TASK_TIM, &timeBaseStructure);

	TIM_SetCounter(MOTION_TASK_TIM, 0);
	TIM_Cmd(MOTION_TASK_TIM, ENABLE);
}


void Serivice_task_init(void){
	//setup timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //SERVICE_TASK_TIM
	
	TIM_DeInit(SERVICE_TASK_TIM);
	//Init SERVICE_TASK_TIM
	TIM_TimeBaseInitTypeDef  timeBaseStructure;
	timeBaseStructure.TIM_Prescaler = (SystemCoreClock / MHz_to_Hz - 1);	//Prescale timer clock to 1MHz - 1us period
	timeBaseStructure.TIM_Period = (10 * 1000) - 1;	//10ms = 10000us
	timeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timeBaseStructure.TIM_RepetitionCounter = 0; //has to be zero to not skip period ticks
	timeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(SERVICE_TASK_TIM, &timeBaseStructure);

	TIM_ClearITPendingBit(SERVICE_TASK_TIM, TIM_IT_Update);
	TIM_ClearFlag(SERVICE_TASK_TIM, TIM_FLAG_Update);
	TIM_ITConfig(SERVICE_TASK_TIM, TIM_IT_Update,ENABLE);

	TIM_SetCounter(SERVICE_TASK_TIM, 0);
	TIM_Cmd(SERVICE_TASK_TIM, ENABLE);
}

void TIM4_IRQHandler(void);//MOTION_TASK_TIM
void TIM2_IRQHandler(void);//SERVICE_TASK_TIM

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

void TIM4_IRQHandler(void) //MOTION_TASK_TIM
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

void TIM2_IRQHandler(void) //SERVICE_TASK_TIM triggering
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
