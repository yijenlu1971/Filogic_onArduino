#include <Adafruit_SPIDevice.h>

typedef struct led_cmd
{
    uint32_t    start;
    uint8_t     led1_global;
    uint8_t     led1_b;
    uint8_t     led1_g;
    uint8_t     led1_r;
    uint8_t     led2_global;
    uint8_t     led2_b;
    uint8_t     led2_g;
    uint8_t     led2_r;
    uint32_t    end;
} led_cmd_t;

#define SPIDEVICE_CS 7
//Adafruit_SPIDevice spi_dev = Adafruit_SPIDevice(SPIDEVICE_CS);
Adafruit_SPIDevice spi_dev = Adafruit_SPIDevice(SPIDEVICE_CS, 6, 8, 9, 100000, SPI_BITORDER_MSBFIRST, SPI_MODE1);

static led_cmd_t led_cmd;
int led1 = 0, led2 = 0x07;

void setup() {
  while (!Serial) { delay(10); }
  Serial.begin(115200);
  Serial.println("Filogic130A RGB ligth test");

  if (!spi_dev.begin()) {
    Serial.println("Could not initialize SPI device");
    while (1);
  }
  
  led_cmd.start      = 0x00000000;
  led_cmd.led1_global = 0xF0;
  led_cmd.led1_b     = 0x00;
  led_cmd.led1_g     = 0x00;
  led_cmd.led1_r     = 0x00;
  led_cmd.led2_global = 0xF0;
  led_cmd.led2_b     = 0x00;
  led_cmd.led2_g     = 0x00;
  led_cmd.led2_r     = 0x00;
  led_cmd.end        = 0xFFFFFFFF;

  spi_dev.write((const uint8_t*)&led_cmd, sizeof(led_cmd), NULL, 0);
}

void loop() {

  led_cmd.led1_b     = ((led1 & 0x01) == 0x01) ? 0x01 : 0x00;
  led_cmd.led1_g     = ((led1 & 0x02) == 0x02) ? 0x01 : 0x00;
  led_cmd.led1_r     = ((led1 & 0x04) == 0x04) ? 0x01 : 0x00;
  if( ++led1 >= 8 ) led1 = 0;

  led2--;
  led_cmd.led2_b     = ((led2 & 0x01) == 0x01) ? 0x01 : 0x00;
  led_cmd.led2_g     = ((led2 & 0x02) == 0x02) ? 0x01 : 0x00;
  led_cmd.led2_r     = ((led2 & 0x04) == 0x04) ? 0x01 : 0x00;
  if( led2 == 0 ) led2 = 0x07;

  spi_dev.write((const uint8_t*)&led_cmd, sizeof(led_cmd), NULL, 0);
  delay(2000);
}

