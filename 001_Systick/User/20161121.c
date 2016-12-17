/******************** HJ ********************
 * Action	：LED
 * MCU		：STM32F103C8T6      
 * Auther	: HuiJiang
 * Date		：20161121 																										  
 *****************************************************/	

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_usart.h"
#include "core_cm3.h"
#include "misc.h"


uint32_t sTicks = 1;                       /* timeTicks counter */

//void Delay(uint32_t nTime)
void SysTick_Handler(void)
{
	sTicks = ~sTicks;
	if(sTicks == 1)
	{ 
		 GPIO_ResetBits(GPIOA, GPIO_Pin_2|GPIO_Pin_3);
	}
	else
  {
		GPIO_SetBits(GPIOA, GPIO_Pin_2|GPIO_Pin_3);
  }
}

void RCC_Configuration(void)
{
	/*Deinitialize the HSEStartUpStatus value*/
	ErrorStatus HSEStartUpStatus;
	/*Deinitialize the RCC registers*/
	RCC_DeInit();
	/* Enable the HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	/* Wait till HSE is ready and if Time out is reached exit */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_ACR_LATENCY_2);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		/* Set PLL clock output to 72MHz using HSE (8MHz) as entry clock */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);
		
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{}
		/* Select the PLL as system clock source */
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08)
		{}
	}
	//RCC_AHBPeriphClockCmd();
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
	//RCC_APB1PeriphClockCmd();
}
void NVIC_Configuration(void)
{
		//NVIC_SetPriority(SysTick_Handler, 3); 
	NVIC_InitTypeDef NVIC_InitStructure;
	#ifdef VECT_TAB_RAM
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//	NVIC_InitStructure.NVIC_IRQChannel = ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}
int main(void)
{
	/*结构体变量的声明，不能放在可执行语句的后面，必须在主函数的开头声明结构体变量*/
	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_Configuration();	
	SysTick_Config(900000);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	/*初始化GPIOA引脚*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	/*调用函数库，使用上面配置的 GPIO_InitStructure 初始化 GPIO*/
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*使引脚输出高电平，点亮LED*/
//	GPIO_SetBits(GPIOA, GPIO_Pin_8);
//	GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
//	Delay(30);
//	RCC_MCOConfig(RCC_MCO_HSE);
//	GPIO_ResetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
//	Delay(30);
//	GPIO_SetBits(GPIOA, GPIO_Pin_2 | GPIO_Pin_3);
	while(1);
}
