#include "Arduino.h"
#include "hal_gpio.h"

#define INT_ENABLE    0

#define LED_BUILTIN   7
#define SW_KEY_2     47
#define SW_KEY_3     49

bool bOn = false;

#if INT_ENABLE
bool bKey2 = false;
void key2_change(void)
{
  bKey2 = true;
}
#endif

void setup() {
  char  buf[32];
  hal_gpio_data_t gpioData;
  hal_gpio_mode_t gpioMode;
  hal_gpio_direction_t gpioDir;

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // show all GPIO mapping
  for(int i = 0; i < HAL_GPIO_MAX; i++)
  {
    hal_gpio_get_function((hal_gpio_pin_t)i, &gpioMode);
    hal_gpio_get_direction((hal_gpio_pin_t)i, &gpioDir);
    // show all GPIO mode and direction
    sprintf(buf, "GPIO%d Mode=%d DIR=%d\n", i, gpioMode, gpioDir);
    Serial.print(buf);
  }

  pinMode(SW_KEY_2, INPUT_PULLUP);
  pinMode(SW_KEY_3, INPUT_PULLUP);

  // Initialize LED pin
  pinMode(6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

#if INT_ENABLE
  attachInterrupt(SW_KEY_2, key2_change, CHANGE);
#endif
}

void loop() {
  hal_gpio_data_t   value;
//  Serial.println(bOn);
#if !INT_ENABLE
  hal_gpio_get_input((hal_gpio_pin_t)SW_KEY_2, &value);
  Serial.print(value);
#else
  Serial.print(bKey2);
  if( bKey2 ) bKey2 = false;
#endif
  Serial.print(",");
  hal_gpio_get_input((hal_gpio_pin_t)SW_KEY_3, &value);
  Serial.println(value);

  /*Serial.print(digitalRead(SW_KEY_2));
  Serial.print(",");  
  Serial.println(digitalRead(SW_KEY_3));*/
  digitalWrite(LED_BUILTIN, bOn ? HIGH : LOW);
  digitalWrite(6, bOn ? HIGH : LOW);
  bOn = !bOn;

  delay(1000);
}
