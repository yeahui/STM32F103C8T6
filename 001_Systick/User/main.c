/******************** HJ ********************
 * Action	£ºSysTick
 * MCU		£ºSTM32F103C8T6      
 * Auther	: HuiJiang
 * Date		£º20161216																										  
 *****************************************************/	

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_tim.h"
#include "core_cm3.h"
#include "misc.h"


uint32_t sTicks = 1;                       /* timeTicks counter */
uint32_t sTime = 1;  

void SysTick_Handler(void)
{
	sTicks = ~sTicks;
	if(sTicks == 1)
	{ 
		 GPIO_ResetBits(GPIOC, GPIO_Pin_15);
	}
	else
  {
		GPIO_SetBits(GPIOC, GPIO_Pin_15);
  }
}

void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08)
		{}
	}
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	#ifdef VECT_TAB_RAM
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		sTime++;
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);
	}
}
void TIM3_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 999;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 71;
//	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
int main(void)
{
	uint16_t sTimes = 1;
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	SysTick_Config(9000000);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	TIM3_Configuration();
	while(1)
	{
		if(sTime == 1000)
		{
			sTime= 0;
			sTimes = ~sTimes;
			if(sTimes == 1)
			{ 
			GPIO_ResetBits(GPIOC, GPIO_Pin_14);
			}
			else
			{
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
			}
		}
	}
}
