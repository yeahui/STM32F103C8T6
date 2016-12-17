/******************** HJ ********************
 * Action	USART_STRING
 * MCU		STM32F103C8T6      
 * Auther	HuiJiang
 * Date		20161203 																										  
 *****************************************************/	

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "core_cm3.h"
#include "misc.h"

void USART_Configuration(void)
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

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}
void Usart_SendByte(USART_TypeDef *pUSARTx, uint8_t ch)
{
	USART_SendData(pUSARTx, ch);
	while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
	{}
}
void Usart_SendString(USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k = 0;
	do{
		Usart_SendByte(pUSARTx, *(str + k - 1));
		k++;
	}while(*(str + k - 1) != '\0');
	while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET)
	{}
}
int main(void)
{

	USART_Configuration();
	Usart_SendString(USART1, "\r\n welcome to here \r\n" "(" __DATE__ "-" __TIME__ ")");

	while(1);
}
