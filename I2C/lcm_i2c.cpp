/**
  ******************************************************************************
  * @file    lcm_i2c.cpp
  * @author  Yi-Jen Lu (yijenlu@yahoo.com)
  * @date    26-Jul-2022
  ******************************************************************************
  */
#include <string.h>
#include "hal_i2c_master.h"
#include "lcm_i2c.h"
#include "Arduino.h"
#include "UARTClass.h"


uint8_t lcmAddr = 0, bBusy = 0;


void LCD_SendInternal(uint8_t lcd_addr, uint8_t data, uint8_t flags)
{
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up | flags | BACKLIGHT | PIN_EN;
	data_arr[1] = up | flags | BACKLIGHT;
	data_arr[2] = lo | flags | BACKLIGHT| PIN_EN;
	data_arr[3] = lo | flags | BACKLIGHT;
	
	if( HAL_I2C_STATUS_OK != hal_i2c_master_send_polling(HAL_I2C_MASTER_1, lcd_addr, data_arr, 4) )
	{
		Serial.print("LCD_SendInternal err!!!\n");
	}
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd)
{
	LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data)
{
	LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void Lcm_Init(uint8_t lcd_addr)
{
	lcmAddr = lcd_addr;
	// 4-bit mode, 2 lines, 5x7 format
	LCD_SendCommand(lcmAddr, 0x30);
	// display & cursor home (keep this!)
	LCD_SendCommand(lcmAddr, 0x02);
	// display on, right shift, underline off, blink off
	LCD_SendCommand(lcmAddr, 0x0C);
	// clear display (optional here)
	LCD_Clear();
}

void LCD_Clear(void)
{
	LCD_SendCommand(lcmAddr, 0x01);
	delay(50);
}

void LCD_SendString(uint8_t line, char *str)
{
	int	len = 0;

	while( bBusy ) delay( 20 );

	bBusy = TRUE;
	LCD_SendCommand(lcmAddr, (line == 0) ? 0x80 : 0xC0);
	while(*str && len < 16)
	{
		LCD_SendData(lcmAddr, (uint8_t)(*str));
		str++;
		len++;
	}

	bBusy = FALSE;
}
