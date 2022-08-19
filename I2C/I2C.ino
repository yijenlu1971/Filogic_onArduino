#include "Arduino.h"
#include "lcm_i2c.h"
#include "sensors_i2c.h"
#include "hal_gpio.h"
#include "hal_i2c_master.h"

#define LED_BUILTIN   7

bool bOn = false;

void setup() {
  char  buf[32];
  hal_gpio_data_t gpioData;
  hal_gpio_mode_t gpioMode;
  hal_gpio_direction_t gpioDir;
  hal_i2c_config_t i2c_cfg;

  // Initialize LED pin
  pinMode(6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // show all GPIO mapping
  /*for(int i = 0; i < HAL_GPIO_MAX; i++)
  {
    hal_gpio_get_function((hal_gpio_pin_t)i, &gpioMode);
    hal_gpio_get_direction((hal_gpio_pin_t)i, &gpioDir);
    // show all GPIO mode and direction
    sprintf(buf, "GPIO%d Mode=%d DIR=%d\n", i, gpioMode, gpioDir);
    Serial.print(buf);
  }*/

  hal_i2c_master_deinit(HAL_I2C_MASTER_1);
  i2c_cfg.frequency = HAL_I2C_FREQUENCY_400K;
  if( HAL_I2C_STATUS_OK != hal_i2c_master_init(HAL_I2C_MASTER_1, &i2c_cfg) )
  {
    Serial.print("hal_i2c_master_init err!!!\n");
  }

  Lcm_Init(0x27);
  Sht_Reset(0x44);
  delay(500);

  Sht_Init(0x44);
  Bh1750_Init(0x23);
  delay(500);
}

void loop() {
  uint8_t  buf[32];
  uint16_t tempVal, humVal, bhVal;

  if( !Bh1750_LightLevel(&bhVal) )
  {
    sprintf((char*)buf, "LX:%5d        ", bhVal);
    LCD_SendString(0, (char*)buf);
  }

  if( !Sht_GetTempHumidity(&tempVal, &humVal) )
  {
    sprintf((char*)buf, "T:%2d.%1dC H:%2d.%1d%% ", tempVal/10, tempVal - (tempVal/10)*10, humVal/10, humVal - (humVal/10)*10);
    LCD_SendString(1, (char*)buf);
  }

  digitalWrite(LED_BUILTIN, bOn ? HIGH : LOW);
  bOn = !bOn;

  delay(1000);
}
