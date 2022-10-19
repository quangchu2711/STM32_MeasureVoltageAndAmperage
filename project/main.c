/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define PAGE_ADDR_127 (0x0801FC00U)
/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
uint32_t setTimeMode[4] = {3, 4, 5, 6};
/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void Flash_WriteData(uint32_t data, uint32_t addr);
void Flash_WriteDataArray(uint32_t dataArr[], uint8_t lenArr, uint32_t startAddr);
void Led_Config(void);
uint32_t Flash_ReadData(uint32_t addr);
/******************************************************************************/
/*                            			MAIN                                      */
/******************************************************************************/
int main(void)
{
	SysTick_Init();
	Serial_Begin(9600);
	Led_Config();
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, (BitAction)1);

  while (1)
  {
			uint8_t i;
			uint32_t data;
			uint32_t addrStart = PAGE_ADDR_127;
			for (i = 0; i < 4; i++)
			{
				data = Flash_ReadData(addrStart);
				Serial_Printf("[%d]\n", data);
				delay_ms(2000);
				addrStart += 4;
			}
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
/******************************************************************************/
/*                            			END CODE                                  */
/******************************************************************************/