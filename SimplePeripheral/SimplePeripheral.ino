/*
  This example configures LinkIt 7697 to act as a simple GATT server with 1 characteristic.

  To use it, open AppInventor project:

    * 

  Build & install it on Android id

  created Mar 2017
*/
#include <LBLE.h>
#include <LBLEPeriphral.h>

// Define a simple GATT service with only 1 characteristic
LBLEService ledService("110C0010-E8F2-537E-4F6C-D104768A1214");
LBLECharacteristicString switchCharacteristic("110C0011-E8F2-537E-4F6C-D104768A1214", LBLE_READ | LBLE_WRITE);

#define LED_BUILTIN   7
#define BUF_SIZE      30

bool bConnStatus = false;
uint8_t rData[BUF_SIZE];

void setup() {

  // Initialize LED pin
  pinMode(6, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  //Initialize serial and wait for port to open:
  Serial.begin(115200);

  // Initialize BLE subsystem
  LBLE.begin();
  while (!LBLE.ready()) {
    delay(100);
  }
  Serial.println("BLE ready");

  Serial.print("Device Address = [");
  Serial.print(LBLE.getDeviceAddress());
  Serial.println("]");

  // configure our advertisement data.
  // In this case, we simply create an advertisement that represents an
  // connectable device with a device name
  LBLEAdvertisementData advertisement;
  advertisement.configAsConnectableDevice("MTK_BLE Lab X");

  // Configure our device's Generic Access Profile's device name
  // Ususally this is the same as the name in the advertisement data.
  LBLEPeripheral.setName("MTK_BLE Lab X");

  // Add characteristics into ledService
  ledService.addAttribute(switchCharacteristic);

  // Add service to GATT server (peripheral)
  LBLEPeripheral.addService(ledService);

  // start the GATT server - it is now 
  // available to connect
  LBLEPeripheral.begin();

  // start advertisment
  LBLEPeripheral.advertise(advertisement);
  Serial.print("conected=");
  Serial.println(bConnStatus);
}

void loop() {
  delay(50);

  if( bConnStatus != LBLEPeripheral.connected() )
  {
    bConnStatus = LBLEPeripheral.connected();
    Serial.print("conected=");
    Serial.println(bConnStatus);
  }
  
  if (switchCharacteristic.isWritten()) {
//    const char value = switchCharacteristic.getValue();
    const String value = switchCharacteristic.getValue();
    switch (value[0]) {
      case '1':
        digitalWrite(LED_BUILTIN, LOW);
        break;
      case '0':
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      default:
        Serial.print("Unknown value written: ");
        Serial.println(value);
        break;
    }
  }
}
