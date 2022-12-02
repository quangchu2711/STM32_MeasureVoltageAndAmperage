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
#define RELAY_POWER_PIN					GPIO_Pin_11
#define RELAY_SOFT_START_PIN		GPIO_Pin_12
#define RELAY_PIN_1 						GPIO_Pin_3
#define RELAY_PIN_2 						GPIO_Pin_4
#define RELAY_PIN_3 						GPIO_Pin_5
#define RELAY_PIN_4 						GPIO_Pin_6
#define BUZZER_CLOCK 						RCC_APB2Periph_GPIOA
#define BUZZER_PORT							GPIOA
#define BUZZER_PIN							GPIO_Pin_8

typedef enum 
{
	INIT_MODE,
	BUTTON1_MODE,
	BUTTON2_MODE,
	BUTTON3_MODE,
	BUTTON4_MODE	
}Button_Mode_t;

TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
__IO uint16_t g_IC3ReadValue1 = 0, g_IC3ReadValue2 = 0;
__IO uint16_t g_CaptureTimerNumber = 0;
__IO uint32_t g_TimeCapture = 0;
__IO float g_Freq = 0;
uint32_t g_SetTimeModeArr[4] = {3, 4, 5, 6};
uint16_t g_AdcValueArr[3];
uint32_t g_TimeMs = 0;
ButtonManagement button1;
ButtonManagement button2;
ButtonManagement button3;
ButtonManagement button4;
uint32_t TimeMillis = 0;
uint8_t stateLed = 0;
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
Button_Mode_t getButtonMode(void);
uint32_t Flash_ReadData(uint32_t addr);
uint16_t Get_Voltage(void);

void Flash_WriteData(uint32_t data, uint32_t addr);
void Flash_WriteDataArray(uint32_t dataArr[], uint8_t lenArr, uint32_t startAddr);
void TIM3_ChannelConfig(void);
void TIM3_ClockConfig(void);
void TIM3_InputCaptureConfig(void);
void TIM3_InterruptConfig(void);
void Relay_Config(void);
void Buzzer_config(void);
void Buzzer_Start(void);
void Buzzer_Stop(void);
void TurnOff_AllDevices(void);

void TurnOff_AllDevices(void)
{
	GPIO_WriteBit(RELAY_PORT, RELAY_POWER_PIN, (BitAction)(1));
	GPIO_WriteBit(RELAY_PORT, RELAY_SOFT_START_PIN, (BitAction)(1));				
	GPIO_WriteBit(RELAY_PORT, RELAY_PIN_1, (BitAction)(1));
	GPIO_WriteBit(RELAY_PORT, RELAY_PIN_2, (BitAction)(1));
	GPIO_WriteBit(RELAY_PORT, RELAY_PIN_3, (BitAction)(1));
	GPIO_WriteBit(RELAY_PORT, RELAY_PIN_4, (BitAction)(1));					
}

uint16_t Get_Voltage(void)
{
	uint16_t a;
	uint16_t data, ADCm, A, B, C, V, max;
	for (a = 0; a < 500; a++)
	{
		data = g_AdcValueArr[0]; //Doc ADC tai chan PA0
		if(data > max)
		{
			ADCm=data;
			max=data;
		}
		delay_ms(1);
	}
	A=ADCm*2048/402;
	B=((A*3.3)/4096);
	//V=((-1.65*821000)+B*821000)/(100*(47/2.2)*1.142);
	C= (B-1.65)/(((1.65+(((ADCm-402)*3.3/4096)))*100*(47/2.2))/821000);
	V = C;
	return V;
}

int main(void)
{
	SysTick_Init();
	Serial_Begin(9600);
	/* System Clocks Configuration */
//	Led_Config();
//	GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)1);
	ADC_Multi_Channel_Config(3, ADC_SampleTime_239Cycles5);
	DMA_Multi_Channel_Config(g_AdcValueArr, 3); /*PA0, PA1, PA2*/
	DMA_Start();
	Button_Config(BUTTON_PORT, BUTTON_PIN_1, &button1);
	Button_Config(BUTTON_PORT, BUTTON_PIN_2, &button2);
	Button_Config(BUTTON_PORT, BUTTON_PIN_3, &button3);
	Button_Config(BUTTON_PORT, BUTTON_PIN_4, &button4);
//	TIM3_InputCaptureConfig();
//	Serial_Printf("Connected");
	LCD_I2C_Configuration();
	LCD_Init();
	Relay_Config();
	Buzzer_config();
	TurnOff_AllDevices();
  while (1)
  {
		LCD_Gotoxy(1, 0);
		LCD_Printf("%d- %d -%d", g_AdcValueArr[0], g_AdcValueArr[2]);
		LCD_Gotoxy(0, 0);
		LCD_Printf("%d", Get_Voltage());
		delay_ms(100);
//		//Test ferquency
//		if (SysTick_Millis() - g_TimeMs > 10)
//		{
//			stateLed = !stateLed;
//			g_TimeMs = SysTick_Millis();
//		}
//		GPIO_WriteBit(GPIOB, GPIO_Pin_5, (BitAction)(stateLed));
		
		//
//		LCD_Gotoxy(1, 0);
//		LCD_Printf("%d Hz", g_Freq);
		//Test button (Done)
		if (Serial_Available())
		{
			uint8_t mode = Serial_Read();
			if (mode == 'O')
			{
				Buzzer_Start();
				GPIO_WriteBit(RELAY_PORT, RELAY_POWER_PIN, (BitAction)(1));
				GPIO_WriteBit(RELAY_PORT, RELAY_SOFT_START_PIN, (BitAction)(1));				
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_1, (BitAction)(1));
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_2, (BitAction)(1));
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_3, (BitAction)(1));
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_4, (BitAction)(1));				
			}
			else	if (mode == 'F')
			{
				GPIO_WriteBit(RELAY_PORT, RELAY_POWER_PIN, (BitAction)(0));
				GPIO_WriteBit(RELAY_PORT, RELAY_SOFT_START_PIN, (BitAction)(0));				
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_1, (BitAction)(0));
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_2, (BitAction)(0));
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_3, (BitAction)(0));
				GPIO_WriteBit(RELAY_PORT, RELAY_PIN_4, (BitAction)(0));				
			}
		}
		Button_Mode_t buttonMode = getButtonMode();
		if (buttonMode != INIT_MODE)
		{
//			Serial_Printf("[Button: %d]\n", buttonMode);	
			LCD_Gotoxy(0, 0);
			LCD_Printf("%d", buttonMode);
			switch (buttonMode)
      {
      	case BUTTON2_MODE:
					Buzzer_Start();
					GPIO_WriteBit(RELAY_PORT, RELAY_POWER_PIN, (BitAction)(1));
					GPIO_WriteBit(RELAY_PORT, RELAY_SOFT_START_PIN, (BitAction)(1));				
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_1, (BitAction)(1));
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_2, (BitAction)(1));
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_3, (BitAction)(1));
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_4, (BitAction)(1));				
				break;
      	case BUTTON3_MODE:
					Buzzer_Stop();
					GPIO_WriteBit(RELAY_PORT, RELAY_POWER_PIN, (BitAction)(0));
					GPIO_WriteBit(RELAY_PORT, RELAY_SOFT_START_PIN, (BitAction)(0));				
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_1, (BitAction)(0));
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_2, (BitAction)(0));
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_3, (BitAction)(0));
					GPIO_WriteBit(RELAY_PORT, RELAY_PIN_4, (BitAction)(0));
      		break;
      }

		}
//		LCD_Gotoxy(0, 0);
//		LCD_Printf("%d", buttonMode);

		
		//Test DMA (Done) 
//		Serial_Printf("[%d - %d - %d]\n", g_AdcValueArr[0], g_AdcValueArr[1], g_AdcValueArr[2]);
//		delay_ms(500);
		
		
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

void TIM3_ClockConfig(void)
{
	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

 void TIM3_ChannelConfig(void)
 {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* TIM3 channel 2 pin (PA.07) configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM3_InterruptConfig(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM3 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM3_InputCaptureConfig(void)
{
	TIM3_ClockConfig();
	TIM3_ChannelConfig();
	TIM3_InterruptConfig();
	/* TIM3 configuration: Input Capture mode ---------------------
	The external signal is connected to TIM3 CH2 pin (PA.07)
	The Rising edge is used as active edge,
	The TIM3 CCR2 is used to compute the frequency value
	------------------------------------------------------------ */
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);	
}

void TIM3_IRQHandler(void)
{ 
	if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) 
 	{
		/* Clear TIM3 Capture compare interrupt pending bit */
	   TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	   if(g_CaptureTimerNumber == 0)
       {
        /* Get the Input Capture value */
        g_IC3ReadValue1 = TIM_GetCapture2(TIM3);
        g_CaptureTimerNumber = 1;
      }
      else if(g_CaptureTimerNumber == 1)
			{
        /* Get the Input Capture value */
        g_IC3ReadValue2 = TIM_GetCapture2(TIM3); 

       /* Capture computation */
       if (g_IC3ReadValue2 > g_IC3ReadValue1)
       {
         g_TimeCapture = (g_IC3ReadValue2 - g_IC3ReadValue1); 
       }
       else
       {
         g_TimeCapture = ((0xFFFF - g_IC3ReadValue1) + g_IC3ReadValue2); 
       }
       /* Frequency computation */ 
			 float prescalerClock = SystemCoreClock/(72);
       g_Freq = prescalerClock / g_TimeCapture;
			 //Serial_Printf("%.2f Hz\n", g_Freq);
				uint16_t hz = (uint16_t)(g_Freq);
				LCD_Gotoxy(0, 0);
				LCD_Printf("%f Hz", hz);

			 g_TimeCapture = 0;
       g_CaptureTimerNumber = 0;
     }
   }
}

void Buzzer_config(void)
{
	RCC_APB2PeriphClockCmd(BUZZER_CLOCK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  BUZZER_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);		
}

void Buzzer_Start(void)
{
	GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, (BitAction)(1));
}

void Buzzer_Stop(void)
{
	GPIO_WriteBit(BUZZER_PORT, BUZZER_PIN, (BitAction)(0));	
}

void Relay_Config(void)
{
	RCC_APB2PeriphClockCmd(RELAY_CLOCK, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  RELAY_POWER_PIN | RELAY_SOFT_START_PIN |  RELAY_PIN_1 | RELAY_PIN_2 | RELAY_PIN_3 | RELAY_PIN_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(RELAY_PORT, &GPIO_InitStructure);	
}