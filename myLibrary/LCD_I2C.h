/**
  ******************************************************************************
  * @file    Lcd_i2c.h
  * @author  Chu Xuan Quang
  * @date    01-02-2022
  * @brief   Lcd_i2c program body
  ******************************************************************************
  * @attention Code can be used for LCD 16x02 and Lcd20x04
  ******************************************************************************
*/

#ifndef LCD_I2C_H
#define LCD_I2C_H
	 
#include "stm32f10x.h"
#include "Delay.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define PCF8574A_Address      	0x27 << 1
#define I2C_Chanel      				I2C2

void LCD_I2C_Configuration(void);
void LCD_Init (void);
void LCD_Write_Byte(uint8_t data);
void LCD_Data_Write(uint8_t data);
void LCD_Cmd_Write(uint8_t data);
void LCD_Send_String (uint8_t *str);
void LCD_Gotoxy(uint8_t x, uint8_t y);
void LCD_Printf(const char* str, ...);
void LCD_Clear(void);

#endif
