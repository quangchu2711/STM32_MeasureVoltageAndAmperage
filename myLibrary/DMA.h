 /**
   ******************************************************************************
   * @file    DMA.H 
   * @author  Chu Xuan Quang
   * @version V1.0.0
   * @date    21-01-2022
   * @brief   DMA program body.
   ******************************************************************************
   * @attention: DAM(ADC1) use 8 channel: PA0->PA7 (SMT32F103C8T6)
	 *	How to use:
	 *		Step 1: ADC_Multi_Channel_Config(uint8_t ADC_channel_number, uint8_t ADC_sampleTime);
	 *		Step 2: DMA_Multi_Channel_Config(uint16_t *ADC_values, uint8_t array_size);
	 *		Step 3: DMA_Start(void);
   ******************************************************************************
   */
#ifndef DMA_H
#define DMA_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

void ADC_Multi_Channel_Config(uint8_t ADC_channel_number, uint8_t ADC_sampleTime);
void DMA_Multi_Channel_Config(uint16_t *ADC_values, uint8_t array_size);
void DMA_Start(void);

#endif
