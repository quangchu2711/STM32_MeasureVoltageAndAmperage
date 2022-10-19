/**
  ******************************************************************************
  * @file    Button.h
  * @author  Chu Xuan Quang
  * @date    17-11-2021
  * @brief   Button program body
  ******************************************************************************
  * @attention
  ******************************************************************************
*/
#ifndef __BUTTON_H
#define __BUTTON_H

/* Includes ------------------------------------------------------------------*/
#include<stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "Delay.h"

#define DEBOUNCE_TIME   50 

#define  IS_PRESS    0x01
#define  WAS_PRESS   0x02
#define  ON_PRESS    0x04
#define  ON_RELEASE  0x08
#define  IS_HOLD     0x10
#define  ON_HOLD     0x20

typedef struct
{
	uint32_t holdTime;
	uint64_t changeTime;
	uint8_t flags;
}ButtonManagement;

void Button_Config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX);
void Button_Update(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX);
bool Button_OnPress(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX);
bool Button_OnHold(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX);
void Button_SetHoldTime (ButtonManagement *ButtonX, uint32_t timeMs);

#endif 
