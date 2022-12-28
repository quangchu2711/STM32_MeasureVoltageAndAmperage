/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "DMA.h"
#include "Button.h"
#include "LCD_I2C.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define PAGE_ADDR_127 					(0x0801FC00U)
#define BUTTON_PORT							GPIOB
#define BUTTON_PIN_1 						GPIO_Pin_5
#define BUTTON_PIN_2 						GPIO_Pin_6
#define BUTTON_PIN_3 						GPIO_Pin_7
#define BUTTON_PIN_4 						GPIO_Pin_8
#define RELAY_CLOCK 						RCC_APB2Periph_GPIOA
#define RELAY_PORT							GPIOA
#define RELAY1_POWER_PIN				GPIO_Pin_11
#define RELAY2_SOFT_START_PIN		GPIO_Pin_12
#define RELAY3_PIN 						  GPIO_Pin_3
#define RELAY4_PIN 						  GPIO_Pin_5
#define RELAY5_PIN 						  GPIO_Pin_6
#define RELAY6_PIN 						  GPIO_Pin_4
#define BUZZER_CLOCK 						RCC_APB2Periph_GPIOA
#define BUZZER_PORT							GPIOA
#define BUZZER_PIN							GPIO_Pin_8

//#define RELAY1_ON_TIME 					
#define RELAY2_OFF_TIME         2000
#define RELAY3_ON_TIME					1000
#define RELAY4_ON_TIME					5000
#define RELAY5_ON_TIME					2000
#define RELAY6_ON_TIME					1000

#define RELAY_ON 								((BitAction)0)
#define RELAY_OFF 							((BitAction)1)
#define MIN_CLOSE_TIME					1
#define MAX_CLOSE_TIME					9

typedef enum 
{
	INIT_MODE,
	BUTTON1_MODE,
	BUTTON2_MODE,
	BUTTON3_MODE,
	BUTTON4_MODE,
	BUTTON3_SETUP_TIME,	
}Button_Mode_t;

typedef enum
{
	Display,
	ModeButton1_on,
	ModeButton1_off,	
	EditTime,
}System_Mode_t;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
uint32_t g_SetTimeModeArr[6];
uint16_t g_AdcValueArr[3];
uint32_t g_TimeMs;
uint8_t g_StatusButton1 = 0;
uint8_t g_Read;
ButtonManagement button1;
ButtonManagement button2;
ButtonManagement button3;
ButtonManagement button4;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
Button_Mode_t getButtonMode(void);
uint32_t Flash_ReadData(uint32_t addr);
uint16_t Get_Voltage(void);
float Get_Amperage(int16_t adcValue);
System_Mode_t Get_SystemMode(void);

void Flash_WriteData(uint32_t data, uint32_t addr);
void Flash_WriteDataArray(uint32_t dataArr[], uint8_t lenArr, uint32_t startAddr);
void Relay_Config(void);
void Flash_SendDataToArray(void);
void TurnOn_AllDevices(void);
void TurnOff_AllDevices(void);
void TurnOn_ModeButton1(void);
void TurnOff_ModeButton1(void);

int main(void)
{
	SysTick_Init();
	Serial_Begin(9600);
	/* System Clocks Configuration */
	ADC_Multi_Channel_Config(3, ADC_SampleTime_239Cycles5);
	DMA_Multi_Channel_Config(g_AdcValueArr, 3); /*PA0, PA1, PA2*/
	DMA_Start();
	Button_Config(BUTTON_PORT, BUTTON_PIN_1, &button1);
	Button_Config(BUTTON_PORT, BUTTON_PIN_2, &button2);
	Button_Config(BUTTON_PORT, BUTTON_PIN_3, &button3);
	Button_Config(BUTTON_PORT, BUTTON_PIN_4, &button4);

	LCD_I2C_Configuration();
	LCD_Init();
	Relay_Config();

	TurnOff_AllDevices();
//	LCD_Clear();
	Flash_SendDataToArray();
	//TurnOn_AllDevices();
	TurnOff_AllDevices();
	
	GPIO_WriteBit(RELAY_PORT, RELAY1_POWER_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY2_SOFT_START_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY3_PIN, RELAY_ON);
	Button_Mode_t buttonSta;
	System_Mode_t sysMode = Display;
	uint8_t staButton1 = 1;
	uint8_t index = 0;
	uint16_t maxVoltage = Get_Voltage();
	float maxI1 = Get_Amperage(g_AdcValueArr[1]);
	float maxI2 = Get_Amperage(g_AdcValueArr[2]);	
	while (1)
  {
		switch (sysMode)
    {
    	case Display:
				sysMode = Get_SystemMode();
				if (SysTick_Millis() - g_TimeMs > 5)
				{
					index += 1;
					if (Get_Voltage() > maxVoltage)
					{
						maxVoltage = Get_Voltage();
					}
					if (Get_Amperage(g_AdcValueArr[1]) >  maxI1)
					{
						maxI1 = Get_Amperage(g_AdcValueArr[1]);
					}
					if (Get_Amperage(g_AdcValueArr[2]) >  maxI2)
					{
						maxI2 = Get_Amperage(g_AdcValueArr[2]);
					}
					g_TimeMs = SysTick_Millis();
				}
				if (index == 100)
				{
					Serial_Printf("Vol: %d - Amp1: %.2f - Amp2: %.2f\n", maxVoltage, maxI1, maxI2);		
					index = 0;
				}
				LCD_Gotoxy(0, 0);
				LCD_Printf("V: %d         ", maxVoltage);
				LCD_Gotoxy(1, 0);
				LCD_Printf("I1:%.2f-I2:%.2f  ", maxI1, maxI2);

    		break;
			case ModeButton1_on:
				TurnOn_ModeButton1();
				sysMode = Display;
				break;
			case ModeButton1_off:
				TurnOff_ModeButton1();
				sysMode = Display;
				break;			
    	case EditTime:
    		break;
    }
  }
}

void Lcd_Display()
{
	LCD_Gotoxy(0, 0);
	LCD_Printf("%d %d", g_AdcValueArr[1], g_AdcValueArr[2]);
	LCD_Gotoxy(1, 0);
	LCD_Printf("%d - %d", Get_Amperage(g_AdcValueArr[1]), Get_Amperage(g_AdcValueArr[2]));
}

float Get_Amperage(int16_t adcValue)
{
    float value;
    value = (((adcValue * 3.3 / 4095.0) - 1.65) / (50.0 * 1.41)) * 700.0;
    return value;
}

uint16_t Get_Voltage(void)
{
		int16_t value;
		value = g_AdcValueArr[0];
    float vol;
    vol = (((value * 3.3/ 4095.0 )- 1.65) * 2.0 * 821000.0) / (100 * 1.414 * 49.2);
    return (uint16_t)vol;
}

void TurnOff_AllDevices(void)
{
	GPIO_WriteBit(RELAY_PORT, RELAY1_POWER_PIN, RELAY_OFF);
	GPIO_WriteBit(RELAY_PORT, RELAY2_SOFT_START_PIN, RELAY_OFF);
	GPIO_WriteBit(RELAY_PORT, RELAY5_PIN, RELAY_OFF);
	GPIO_WriteBit(RELAY_PORT, RELAY6_PIN, RELAY_OFF);
	GPIO_WriteBit(RELAY_PORT, RELAY4_PIN, RELAY_OFF);
	GPIO_WriteBit(RELAY_PORT, RELAY3_PIN, RELAY_OFF);
}

void TurnOn_AllDevices(void)
{
	GPIO_WriteBit(RELAY_PORT, RELAY1_POWER_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY2_SOFT_START_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY5_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY6_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY4_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY3_PIN, RELAY_ON);
}

void Flash_SendDataToArray(void)
{
		uint8_t i;
		uint32_t data;
		uint32_t addrStart = PAGE_ADDR_127;
		for (i = 0; i < 6; i++)
		{
			data = Flash_ReadData(addrStart);
			g_SetTimeModeArr[i] = data;
			delay_ms(100);
			addrStart += 4;
		}	
}

Button_Mode_t getButtonMode(void)
{
	Button_Mode_t buttonMode = INIT_MODE;
	
	if (Button_OnPress(BUTTON_PORT, BUTTON_PIN_1, &button1))
	{
			buttonMode = BUTTON1_MODE;
	}
	else if (Button_OnPress(BUTTON_PORT, BUTTON_PIN_2, &button2))
	{
			buttonMode = BUTTON2_MODE;
	}
	else if (Button_OnPress(BUTTON_PORT, BUTTON_PIN_3, &button3))
	{
			buttonMode = BUTTON3_MODE;
	}
	else if (Button_OnPress(BUTTON_PORT, BUTTON_PIN_4, &button4)) 
	{
			buttonMode = BUTTON4_MODE;
	}
	
	if (Button_OnHold(BUTTON_PORT, BUTTON_PIN_3, &button3))
	{
			buttonMode = BUTTON3_SETUP_TIME;
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

void Relay_Config(void)
{
	RCC_APB2PeriphClockCmd(RELAY_CLOCK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  RELAY1_POWER_PIN | RELAY2_SOFT_START_PIN |  RELAY5_PIN | RELAY6_PIN | RELAY4_PIN | RELAY3_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RELAY_PORT, &GPIO_InitStructure);	
}

System_Mode_t Get_SystemMode(void)
{
	System_Mode_t sysMode = Display;
	if (Serial_Available() > 0)
	{
		g_Read = Serial_Read();
		if (g_Read == 'A')
		{
			sysMode = ModeButton1_on;
		}
		else if (g_Read == 'B')
		{
			sysMode = ModeButton1_off;
		}
	}
	else if (getButtonMode() == BUTTON3_SETUP_TIME)
	{
		sysMode = EditTime;
	}
	else if (getButtonMode() == BUTTON1_MODE)
	{
		g_StatusButton1 = !g_StatusButton1;
		if (g_StatusButton1 == 1)
		{
			sysMode = ModeButton1_on;
		}
		else
		{
			sysMode = ModeButton1_off;
		}
	}

	return sysMode;
}

void TurnOn_ModeButton1(void)
{
	GPIO_WriteBit(RELAY_PORT, RELAY1_POWER_PIN, RELAY_ON);
	delay_ms(1000);
	GPIO_WriteBit(RELAY_PORT, RELAY6_PIN, RELAY_ON);
	delay_ms(1000);
	GPIO_WriteBit(RELAY_PORT, RELAY5_PIN, RELAY_ON);
	delay_ms(5000);
	GPIO_WriteBit(RELAY_PORT, RELAY4_PIN, RELAY_ON);
	GPIO_WriteBit(RELAY_PORT, RELAY3_PIN, RELAY_ON);	
	delay_ms(3000);
	GPIO_WriteBit(RELAY_PORT, RELAY2_SOFT_START_PIN, RELAY_ON);
}

void TurnOff_ModeButton1(void)
{
	GPIO_WriteBit(RELAY_PORT, RELAY6_PIN, RELAY_OFF);
	delay_ms(2000);
	GPIO_WriteBit(RELAY_PORT, RELAY5_PIN, RELAY_OFF);
	delay_ms(2000);
	GPIO_WriteBit(RELAY_PORT, RELAY4_PIN, RELAY_OFF);
	delay_ms(2000);
	GPIO_WriteBit(RELAY_PORT, RELAY3_PIN, RELAY_OFF);
	delay_ms(2000);
	GPIO_WriteBit(RELAY_PORT, RELAY2_SOFT_START_PIN, RELAY_OFF);
	delay_ms(2000);
	GPIO_WriteBit(RELAY_PORT, RELAY1_POWER_PIN, RELAY_OFF);
}