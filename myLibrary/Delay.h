/**
  ******************************************************************************
  * @file    Delay.h
  * @author  Chu Xuan Quang
  * @date    15-11-2021
  * @brief   Delay program body
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#ifndef __DELAY_H
#define __DELAY_H

//#ifdef __cplusplus
// extern "C" {
//#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


void SysTick_Init(void);
void SysTick_Handler(void);
uint64_t SysTick64(void);
uint32_t SysTick32(void);
uint32_t SysTick24(void);
uint64_t SysTick_Millis(void);
uint64_t SysTick_Micros(void);
void delay_us(unsigned long us);
void delay_ms(unsigned long ms);


//#ifdef __cplusplus
//}
//#endif

#endif /* __MISC_H */
