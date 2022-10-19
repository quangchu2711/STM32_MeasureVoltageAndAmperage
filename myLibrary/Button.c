#include "Button.h"

void Button_Config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX)
{
	//Set up pin button
	GPIO_InitTypeDef GPIO_Struct;
	if (GPIOx == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	else if (GPIOx == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if (GPIOx == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_Struct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Struct.GPIO_Pin =  GPIO_Pin;
	GPIO_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOx, &GPIO_Struct);
	
	//Set up ButtonManagement
	ButtonX->changeTime = 0;
	ButtonX->flags = 0x00;
	ButtonX->holdTime = 1000;
}

void Button_Update(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX)
{
	//Read pin state to IS_PRESS
	uint64_t interval = SysTick_Millis() - ButtonX->changeTime;
	if (interval > DEBOUNCE_TIME)
	{
		if (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
		{
			ButtonX->flags |= IS_PRESS;
		}
		else
		{
			ButtonX->flags &= ~IS_PRESS;
		}
	}
	
	//Detect changes
	switch (ButtonX->flags & (WAS_PRESS | IS_PRESS))
	{
		case WAS_PRESS:
			ButtonX->flags = ON_RELEASE;
			ButtonX->changeTime = SysTick_Millis();
		break;
		
		case IS_PRESS:
			ButtonX->flags |= IS_PRESS | ON_PRESS | WAS_PRESS;
			ButtonX->changeTime = SysTick_Millis();
			interval = 0;
		break;
	}
	
	//Checking Hold state
	if ((ButtonX->flags & IS_PRESS) &&       //is press state         
		(!(ButtonX->flags & IS_HOLD)) &&       //is hod not set
		(interval > ButtonX->holdTime))        //long hold flags
	{
		ButtonX->flags |= IS_HOLD | ON_HOLD; 
	}
}

bool Button_OnPress(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX)
{
	Button_Update(GPIOx, GPIO_Pin, ButtonX);
  bool result = ButtonX->flags & ON_PRESS;
  ButtonX->flags &= ~ON_PRESS;                    // Reset event flag after checking
  return result;
}

bool Button_OnHold(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, ButtonManagement *ButtonX)
{
	Button_Update(GPIOx, GPIO_Pin, ButtonX);
  bool result = ButtonX->flags & ON_HOLD;
	ButtonX->flags &= ~ON_HOLD;                    // Reset event flag after checking
  return result;
}

void Button_SetHoldTime (ButtonManagement *ButtonX, uint32_t timeMs)
{
	ButtonX->holdTime = timeMs;
}
