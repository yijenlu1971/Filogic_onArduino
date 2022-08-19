/**
  ******************************************************************************
  * @file    sht30_i2c.h
  * @author  Yi-Jen Lu
  * @date    20-Jun-2022
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHT_I2C_H
#define __SHT_I2C_H

int Sht_GetTempHumidity(uint16_t *tempVal, uint16_t *humVal);
void Sht_Reset(uint8_t sht_addr);
void Sht_Init(uint8_t sht_addr);
void Sht_GetStatus(uint16_t *status);
void Bh1750_Init(uint8_t bh_addr);
int Bh1750_LightLevel(uint16_t *pVal);

#endif /* __SHT_I2C_H */  
