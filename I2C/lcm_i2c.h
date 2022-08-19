/**
  ******************************************************************************
  * @file    lcm_i2c.h
  * @author  Yi-Jen Lu
  * @date    31-Mar-2021
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCM_I2C_H
#define __LCM_I2C_H

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

void Lcm_Init(uint8_t lcd_addr);
void LCD_Clear(void);
void LCD_SendString(uint8_t line, char *str);

#endif /* __LCM_I2C_H */  
