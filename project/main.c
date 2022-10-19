/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "DMA.h"
#include "Button.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define PAGE_ADDR_127 (0x0801FC00U)
#define PORT_BUTTON	GPIOA
#define PIN_BUTTON_1 GPIO_Pin_2
#define PIN_BUTTON_2 GPIO_Pin_3
#define PIN_BUTTON_3 GPIO_Pin_4
#define PIN_BUTTON_4 GPIO_Pin_5

typedef enum 
{
	INIT_MODE,
	BUTTON1_MODE,
	BUTTON2_MODE,
	BUTTON3_MODE,
	BUTTON4_MODE	
}Button_Mode_t;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
uint32_t g_SetTimeModeArr[4] = {3, 4, 5, 6};
uint16_t g_AdcValueArr[2];
uint32_t g_TimeMs = 0;
ButtonManagement button1;
ButtonManagement button2;
ButtonManagement button3;
ButtonManagement button4;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
Button_Mode_t getButtonMode(void);
uint32_t Flash_ReadData(uint32_t addr);

void Flash_WriteData(uint32_t data, uint32_t addr);
void Flash_WriteDataArray(uint32_t dataArr[], uint8_t lenArr, uint32_t startAddr);
void Led_Config(void);

int main(void)
{
	SysTick_Init();
	Serial_Begin(9600);
	Led_Config();
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)1);
	ADC_Multi_Channel_Config(2, ADC_SampleTime_239Cycles5);
	DMA_Multi_Channel_Config(g_AdcValueArr, 2); /*PA0, PA1*/
	DMA_Start();
	Button_Config(PORT_BUTTON, PIN_BUTTON_1, &button1);
	Button_Config(PORT_BUTTON, PIN_BUTTON_2, &button2);
	Button_Config(PORT_BUTTON, PIN_BUTTON_3, &button3);
	Button_Config(PORT_BUTTON, PIN_BUTTON_4, &button4);
	
  while (1)
  {
//		//Test button (Done)
//		Button_Mode_t buttonMode = getButtonMode();
//		if (buttonMode != INIT_MODE)
//		{
//			Serial_Printf("[Button: %d]\n", buttonMode);	
//		}
		
		//Test DMA (Done) 
//		Serial_Printf("[%d - %d]\n", g_AdcValueArr[0], g_AdcValueArr[1]);
//		delay_ms(200);
		
		
		//Test Flash (Done) 		
//			uint8_t i;
//			uint32_t data;
//			uint32_t addrStart = PAGE_ADDR_127;
//			for (i = 0; i < 4; i++)
//			{
//				data = Flash_ReadData(addrStart);
//				Serial_Printf("[%d]\n", data);
//				delay_ms(2000);
//				addrStart += 4;
//			}
  }
}

Button_Mode_t getButtonMode(void)
{
	Button_Mode_t buttonMode = INIT_MODE;
	
	if (Button_OnPress(GPIOA, PIN_BUTTON_1, &button1))
	{
			buttonMode = BUTTON1_MODE;
	}
	else if (Button_OnPress(GPIOA, PIN_BUTTON_2, &button2))
	{
			buttonMode = BUTTON2_MODE;
	}
	else if (Button_OnPress(GPIOA, PIN_BUTTON_3, &button3))
	{
			buttonMode = BUTTON3_MODE;
	}
	else if (Button_OnPress(GPIOA, PIN_BUTTON_4, &button4)) 
	{
			buttonMode = BUTTON4_MODE;
	}
	
	return buttonMode;
}
uint32_t Flash_ReadData(uint32_t addr)
{
	uint32_t data;
	
	data = *(__IO uint32_t *)(addr);
	
	return data;
}

void Flash_WriteData(uint32_t data, uint32_t addr)
{
	FLASH_Unlock();
	while(FLASH_ErasePage(addr) != FLASH_COMPLETE);
	while(FLASH_ProgramWord(addr, data) != FLASH_COMPLETE);
	FLASH_Lock();
}

void Flash_WriteDataArray(uint32_t dataArr[], uint8_t lenArr, uint32_t startAddr)
{
		uint8_t i;
	
		FLASH_Unlock();
		while(FLASH_ErasePage(startAddr) != FLASH_COMPLETE);
		for (i = 0; i < lenArr; i++)
		{
			while(FLASH_ProgramWord(startAddr, dataArr[i]) != FLASH_COMPLETE);	
			startAddr += 4U;
		}
		FLASH_Lock();
}

void Led_Config(void)
{
	//Enable clock GPIOC
	RCC->APB2ENR |= (1 << 4);
	
	//PC13, mode:output, pushpull, max spedd 10Mhz
	GPIOC->CRH &= ~(0xf << 20);
	GPIOC->CRH |= (1 << 20);	
}
