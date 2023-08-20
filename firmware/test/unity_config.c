/*
 Copyright (c) 2014-present PlatformIO <contact@platformio.org>

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
**/

#include <stdio.h>
#include "unity_config.h"
#ifdef USE_STDPERIPH_DRIVER
#include "stm32f10x.h"
#endif
extern void initialise_monitor_handles(void);


void unityOutputStart(void)
{
	#ifdef DEBUG
	initialise_monitor_handles(); //semihosting
	#elif USE_STDPERIPH_DRIVER
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIOB, AFIO and USART1 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

	/* Remap USART1 from PA9/PA10 to PB6/PB7 */
	// GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	#endif
}

void unityOutputChar(char c)
{
	#ifdef DEBUG
    (void) putchar(c);
	#elif USE_STDPERIPH_DRIVER
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, (uint16_t)c);
	#endif
}

void unityOutputFlush(void)
{
	#ifdef DEBUG
    (void) fflush(stdout);
	#elif USE_STDPERIPH_DRIVER
	USART_DeInit(USART1);
	#endif
}

void unityOutputComplete(void){
	//unused
}
