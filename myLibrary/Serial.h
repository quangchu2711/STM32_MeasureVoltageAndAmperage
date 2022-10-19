/**
  ******************************************************************************
  * @file    Serial.h
  * @author  Chu Xuan Quang
  * @date    19-1-2022
  * @brief   Serial program body
  ******************************************************************************
  * @attention UART1: PA9(TX) AND PA10(RX)
  ******************************************************************************
*/
#ifndef SERIAL_H
#define SERIAL_H
#include <stdarg.h>
#include <stdint.h>
#include "stm32f10x.h"
#include "RingBuffer.h"

void Serial_Begin (uint32_t baud);
uint8_t Serial_Read(void);
uint16_t Serial_Available(void);
void Serial_Printf(const char* str, ...);
//UART1
void UART_SendChar(USART_TypeDef *USARTx, char data);
void UART_SendString(USART_TypeDef *USARTx, char *Str);
uint8_t USART_GetChar(USART_TypeDef* USARTx);

#endif
