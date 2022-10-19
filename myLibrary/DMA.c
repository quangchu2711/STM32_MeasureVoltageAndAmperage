#include "DMA.h"

//#define ARRAYSIZE 8
#define ADC1_DR    ((uint32_t)0x4001244C)
//volatile uint16_t ADC_values[ARRAYSIZE];

void DMA_Start(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC_Multi_Channel_Config(uint8_t ADC_channel_number, uint8_t ADC_sampleTime)
{
	//--Enable ADC1 and GPIOA--
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure; //Variable used to setup the GPIO pins
	//==Configure ADC pins (PA0 -> Channel 0 to PA7 -> Channel 7) as analog inputs==
	
	uint16_t GPIO_Pin_X = 0x0000;
	uint8_t i_shift_left = 0;
	do
	{
		GPIO_Pin_X |= ((uint16_t)(0x0001 << i_shift_left));
		i_shift_left += 1;
	}
	while (i_shift_left < ADC_channel_number);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_X;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_InitTypeDef ADC_InitStructure;
	//ADC1 configuration

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//We will convert multiple channels
	FunctionalState x_state = ENABLE;
	if (ADC_channel_number == 1) x_state = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = x_state;
	//select continuous conversion mode
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//channels conversion
	ADC_InitStructure.ADC_NbrOfChannel = ADC_channel_number;
	//load structure values to control and status registers
	ADC_Init(ADC1, &ADC_InitStructure);
	//configure each channel
	uint8_t channel_shift = 0;
	uint8_t rank = 1;
	do
	{
		ADC_RegularChannelConfig(ADC1, (uint8_t)(ADC_Channel_0 + channel_shift), rank, ADC_sampleTime);
		rank += 1;
		channel_shift += 1;
	}
	while (channel_shift < ADC_channel_number);
	//Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	//enable DMA for ADC
	ADC_DMACmd(ADC1, ENABLE);
	//Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);
	//Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));
	//Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	//Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));
}

void DMA_Multi_Channel_Config(uint16_t *ADC_values, uint8_t array_size)
{
	//enable DMA1 clock
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	//create DMA structure
	DMA_InitTypeDef  DMA_InitStructure;
	//reset DMA1 channe1 to default values;
	DMA_DeInit(DMA1_Channel1);
	//channel will be used for memory to memory transfer
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	//setting normal mode (non circular)
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	//medium priority
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	//source and destination data size word=32bit
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	//automatic memory destination increment enable.
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//source address increment disable
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	//chunk of data to be transfered
	DMA_InitStructure.DMA_BufferSize = array_size;
	//source and destination start addresses
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_values;
	//send values to DMA registers
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
//  //Enable DMA1 Channel Transfer Complete interrupt
//	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
//	NVIC_InitTypeDef NVIC_InitStructure;
//	//Enable DMA1 channel IRQ Channel */
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

//void DMA1_Channel1_IRQHandler ()
//{
//	if (DMA_GetITStatus (DMA_IT_TC)) 
//	{

//		DMA_ClearITPendingBit (DMA_IT_TC);
//	}
//}
