#include "Serial.h"
#define buff_len 64
uint8_t Rx_BufArr[buff_len];
RING_BUFFER ring_buffer;

void Serial_Begin (uint32_t baud)
{
	//Config ringbuffer
	RB_Init(&ring_buffer, Rx_BufArr, buff_len);
	
	//Config UART1
	GPIO_InitTypeDef    GPIO_InitStructure;
	USART_InitTypeDef   USART_InitStructure;
	NVIC_InitTypeDef 		NVIC_InitStruct;

	// Step 1: Initialize GPIO for Tx and Rx pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE ); /*enable clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
	/* 
			USART1_Rx : PA10  input floating 
			USART1_Tx : PA9  alternate function push-pull
	*/  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure );

	// Step 2: Initialize USART1
	USART_InitStructure.USART_BaudRate            = baud;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE); //kich hoat usart1
	
	// Step 3: Enable USART receive interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	// Step 4: Initialize NVIC for USART IRQ
	// Set NVIC prority group to group 4 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	// Set System Timer IRQ at higher priority
	NVIC_SetPriority(SysTick_IRQn, 0);
	// Set USART1 IRQ at lower priority
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
//Note: Initialize NVIC for USART IRQ use register
//	USART1->SR = (uint16_t)~USART_IT_RXNE;    			/*Clear interrupt flag for first use*/
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  /*Allow interrupt receive*/
//	NVIC_EnableIRQ(USART1_IRQn);              			/*Enable NVIC UART1*/
}

uint8_t Serial_Read(void)
{
	uint8_t data_read;
	RB_Pop(&ring_buffer, &data_read);
	return data_read;
}

uint16_t Serial_Available(void)
{
	uint16_t num_bytes = RB_Available(&ring_buffer);
	return num_bytes;
}

void Serial_Printf(const char* str, ...)
{
	char stringArray[100];
  va_list args;
  va_start(args, str);
  vsprintf(stringArray, str, args);
  va_end(args);
	UART_SendString(USART1,stringArray);
}
void UART_SendChar(USART_TypeDef *USARTx, char data)
{
    
	USARTx->DR = 0x00;
	USART_SendData(USARTx, data);
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}

void UART_SendString(USART_TypeDef *USARTx, char *Str)
{
	while(*Str)
	{
			UART_SendChar(USARTx, *Str);
			Str++;
	}
}

uint8_t USART_GetChar(USART_TypeDef* USARTx)
{
	uint8_t Data;
	while(USART_GetFlagStatus(USARTx, USART_FLAG_RXNE) == RESET);
	Data = (uint8_t)(USART1->DR & (uint16_t)0x01FF);
	return Data;
}

void USART1_IRQHandler()
{
	uint8_t char_receive = USART_GetChar(USART1);
	RB_Push(&ring_buffer, char_receive);
}
