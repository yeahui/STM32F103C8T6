/******************** HJ ********************
 * Action	：ADC_Temp
 * MCU		：STM32F103C8T6      
 * Auther	: HuiJiang
 * Date		：20161206 																										  
 *****************************************************/	

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "core_cm3.h"
#include "misc.h"
#include "stdio.h"

 
#define ADC1_DR_Address    ((u32)0x4001244C)

__IO u16 ADC_ConvertedValue;
__IO u16 ADC_tempValueLocal;
__IO u16 Current_Temp;
//温度为25摄氏度时的电压值
__IO u16 V25 = 0x6E2;
//每摄氏度4.35mV对应值
__IO u16 Avg_Slope = 0x05;

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

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t)ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	return(ch);
}
int fgetc(FILE *f)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return(int)USART_ReceiveData(USART1);
}
void USART_Configuration(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//CONNECT RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//CONNECT TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能MDA1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	DMA_DeInit(DMA1_Channel1);  //指定DMA通道
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;//设置DMA外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	//设置DMA内存地址，ADC转换结果直接放入该地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设为设置为数据传输的来源
  DMA_InitStructure.DMA_BufferSize = 1;	//DMA缓冲区设置为1；
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_Cmd(DMA1_Channel1, ENABLE); 
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
//	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_55Cycles5);
	ADC_TempSensorVrefintCmd(ENABLE);
	
//	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);	 //使能ADC的DMA
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}
void Delay(uint32_t nCount)
{
	for(;nCount != 0;nCount--){
	}
}
int main(void)
{
	RCC_Configuration();
	USART_Configuration(115200);
	ADC_Config();
	
	printf("\r\n huijiang \r\n");

	while(1)
	{
		ADC_tempValueLocal = ADC_ConvertedValue;
		Delay(0xffffee);
		Current_Temp=(V25-ADC_tempValueLocal)/Avg_Slope+25;
		printf("\r\n The current temperature = %d \r\n", Current_Temp);
	}
}
