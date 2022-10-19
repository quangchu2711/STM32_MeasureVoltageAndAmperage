#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"

#define PAGE_ADDR_127 (0x0801FC00U)

void Led_Config(void);
void Flash_WriteData(uint32_t data, uint32_t addr);
uint32_t Flash_ReadData(uint32_t addr);
uint32_t cnt = 0;
uint32_t readData;

int main(void)
{
	SysTick_Init();
	//Led_Config();
	Serial_Begin(9600);
	Led_Config();
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)1);
	//Flash_WriteData(cnt, PAGE_ADDR_127);

  while (1)
  {
		readData = Flash_ReadData(PAGE_ADDR_127);
		Serial_Printf("%d\n", readData);
		cnt++;
		if ((cnt % 5) == 0)
		{
			Flash_WriteData(cnt, PAGE_ADDR_127);
		}
		delay_ms(1000);
  }
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

void Led_Config(void)
{
	//Enable clock GPIOC
	RCC->APB2ENR |= (1 << 4);
	
	//PC13, mode:output, pushpull, max spedd 10Mhz
	GPIOC->CRH &= ~(0xf << 20);
	GPIOC->CRH |= (1 << 20);	
	
	GPIO_InitTypeDef    GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE ); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure );	
}

