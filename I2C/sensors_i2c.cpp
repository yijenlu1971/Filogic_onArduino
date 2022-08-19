/**
  ******************************************************************************
  * @file    sensors_i2c.cpp : SHT30 sensor & BH1750 sensor
  * @author  Yi-Jen Lu (yijenlu@yahoo.com)
  * @date    26-Jul-2022
  ******************************************************************************
  */
#include <string.h>
#include "hal_i2c_master.h"
#include "Arduino.h"
#include "UARTClass.h"


uint8_t shtAddr = 0, bhAddr = 0;


int Sht_GetTempHumidity(uint16_t *tempVal, uint16_t *humVal)
{
	uint8_t data_arr[6];
	uint32_t temp;
	
	///// for periodic mode
	data_arr[0] = 0xE0;
	data_arr[1] = 0x00;
	if( HAL_I2C_STATUS_OK != hal_i2c_master_send_polling(HAL_I2C_MASTER_1, shtAddr, data_arr, 2) )
	{
		Serial.println("Sht_GetTempHumidity err!!!");
	}
	
	if( HAL_I2C_STATUS_OK != hal_i2c_master_receive_polling(HAL_I2C_MASTER_1, shtAddr, data_arr, 6) )
	{
		Serial.println("Sht_GetTempHumidity err!!!");
		return -1;
	}
	
	temp = data_arr[0];
	temp = ((1750 * ((temp << 8) + data_arr[1])) / 65535);
	*tempVal = (uint16_t) (-450 + temp);
	
	temp = data_arr[0];
	temp = (1000 * ((temp << 8) + data_arr[1])) / 65535;
	*humVal = (uint16_t)temp;

//	Serial.print("SHT val: %02x%02x:%02x (%d), %02x%02x:%02x (%d)\n", data_arr[0], data_arr[1], data_arr[2], *tempVal,
//		data_arr[3], data_arr[4], data_arr[5], *humVal);
	return 0;
}

void Sht_GetStatus(uint16_t *status)
{
	uint8_t data_arr[3];
	char buf[40];

	data_arr[0] = 0xF3;
	data_arr[1] = 0x2D;
	if( HAL_I2C_STATUS_OK != hal_i2c_master_send_polling(HAL_I2C_MASTER_1, shtAddr, data_arr, 2) )
	{
		Serial.println("Sht_GetStatus err!!!");
	}
	else
	{
		if( HAL_I2C_STATUS_OK != hal_i2c_master_receive_polling(HAL_I2C_MASTER_1, shtAddr, data_arr, 3) )
		{
			Serial.println("Sht_GetStatus err!!!");
		}
		else
		{
      sprintf(buf, "SHT status: %02x%02x:%02x\n", data_arr[0], data_arr[1], data_arr[2]);
			Serial.print(buf);
		}
	}
}

void Sht_Reset(uint8_t sht_addr)
{
	uint8_t data_arr[2] = {0x30, 0xA2};

	shtAddr = sht_addr;
	
	if( HAL_I2C_STATUS_OK != hal_i2c_master_send_polling(HAL_I2C_MASTER_1, shtAddr, data_arr, 2) )
	{
		Serial.println("Sht_Reset err!!!");
	}
}

void Sht_Init(uint8_t sht_addr)
{
//	uint8_t data_arr[2] = {0x24, 0x16}; // one shot
	uint8_t data_arr[2] = {0x21, 0x30}; // periodic

	shtAddr = sht_addr;
	
	if( HAL_I2C_STATUS_OK != hal_i2c_master_send_polling(HAL_I2C_MASTER_1, shtAddr, data_arr, 2) )
	{
		Serial.println("Sht_Init err!!!");
	}
}


void Bh1750_Init(uint8_t bh_addr)
{
	// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
	uint8_t data_arr[1] = {0x11};

	bhAddr = bh_addr;
	
	if( HAL_I2C_STATUS_OK != hal_i2c_master_send_polling(HAL_I2C_MASTER_1, bhAddr, data_arr, 1) )
	{
		Serial.print("Bht_Init err!!!\n");
	}
}

int Bh1750_LightLevel(uint16_t *pVal)
{
	uint8_t data_arr[4];
	uint16_t level;

	if( HAL_I2C_STATUS_OK != hal_i2c_master_receive_polling(HAL_I2C_MASTER_1, bhAddr, data_arr, 2) )
	{
		Serial.print("Bh1750_LightLevel err!!!\n");
		return -1;
	}
	
	level = data_arr[0];
	level = (level << 8) | data_arr[1];
	*pVal = level / 1.2;
	
//	Serial.print("Bh1750_LightLevel=%d. (%d)\n", *pVal, level);
	return 0;
}
