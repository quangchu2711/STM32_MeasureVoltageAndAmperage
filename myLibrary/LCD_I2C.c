#include "LCD_I2C.h"

void LCD_Write_Byte(uint8_t data)
{
	/* Send START condition */
	I2C_GenerateSTART(I2C_Chanel, ENABLE);
	/* Test on EV5 and clear it */
	while (!I2C_CheckEvent(I2C_Chanel, I2C_EVENT_MASTER_MODE_SELECT));
	/* Send PCF8574A address for write */
	I2C_Send7bitAddress(I2C_Chanel, PCF8574A_Address, I2C_Direction_Transmitter);
	/* Test on EV6 and clear it */
	while (!I2C_CheckEvent(I2C_Chanel, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	/* Send the byte to be written */
	I2C_SendData(I2C_Chanel, data);
	/* Test on EV8 and clear it */
	while (!I2C_CheckEvent(I2C_Chanel, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	/* Send STOP condition */
	I2C_GenerateSTOP(I2C_Chanel, ENABLE);		
}

void LCD_I2C_Configuration(void)
{
	GPIO_InitTypeDef 					GPIO_InitStructure;
	I2C_InitTypeDef						I2C_InitStructure;  
	
	// cap clock cho ngoai vi va I2C
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);							// su dung kenh I2C2 cua STM32
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// cau hinh chan SDA va SCL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;						//PB10 - SCL, PB11 - SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// cau hinh I2C
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0; // 
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;       
	I2C_InitStructure.I2C_ClockSpeed = 100000;										
	I2C_Init(I2C2, &I2C_InitStructure);
	// cho phep bo I2C hoat dong
	I2C_Cmd(I2C2, ENABLE);

}

void LCD_Init (void)
{
	/* Set 4-bits interface */
	LCD_Cmd_Write(0x33);		 
	delay_ms(10);
	LCD_Cmd_Write(0x32);
	delay_ms(50);
	/* Start to set LCD function */
	LCD_Cmd_Write(0x28);
	delay_ms(50);	
	/* clear LCD */
	LCD_Cmd_Write(0x01);
	delay_ms(50);
	/* wait 60ms */
	
	/* set entry mode */
	LCD_Cmd_Write(0x06);	delay_ms(50);;
	/* set display to on */	
	LCD_Cmd_Write(0x0C);	delay_ms(50);;
	/* move cursor to home and set data address to 0 */
	LCD_Cmd_Write(0x02);	delay_ms(50);
}
void LCD_Data_Write(uint8_t data)
{
	uint8_t data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x0d;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0d;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	for(i = 0;i<4;i++)
	{
		LCD_Write_Byte(data_t[i]);
	}

}

void LCD_Cmd_Write(uint8_t data)
{
  uint8_t data_u, data_l;
	uint8_t data_t[4],i=0;
	data_u = data&0xf0;
	data_l = (data<<4)&0xf0;
	data_t[0] = data_u|0x04;  		//en=1, rs=0
	data_t[1] = data_u;  					//en=0, rs=0
	data_t[2] = data_l|0x04;  		//en=1, rs=0
	data_t[3] = data_l|0x08;  		//en=0, rs=0
	for(i = 0;i<4;i++)
	{
		LCD_Write_Byte(data_t[i]);
	}

}

void LCD_Send_String (uint8_t *str)
{
	while (*str) LCD_Data_Write (*str++);
}

void LCD_Clear(void)
{
	LCD_Cmd_Write(0x01);
	delay_ms(10);
}

//x: row
//y: col
void LCD_Gotoxy(uint8_t x, uint8_t y)
{
	switch(x)
	{
		case 0: LCD_Cmd_Write ((0x80|0x00) + y); break;
		case 1: LCD_Cmd_Write ((0x80|0x40) + y); break;
		case 2: LCD_Cmd_Write ((0x80|0x14) + y); break;
		case 3: LCD_Cmd_Write ((0x80|0x54) + y); break;
	}
}

void LCD_Printf(const char* str, ...)
{
  char stringArray[17];
	
  va_list args;
  va_start(args, str);
  vsprintf(stringArray, str, args);
  va_end(args);
	
  for(uint8_t i=0;  i<strlen(stringArray) && i< 20; i++)
  {
    LCD_Data_Write((uint8_t)stringArray[i]);
  }
}

