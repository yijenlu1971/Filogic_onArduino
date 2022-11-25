/*
  This example scans nearby BLE peripherals and prints the peripherals found.

  created Mar 2017 by MediaTek Labs
*/

#include <LBLE.h>
#include <LBLECentral.h>

#define TARGET_NAME   "MTK_BLE Lab X"
#define SERVICE_UUID  "110C0010-E8F2-537E-4F6C-D104768A1214"
#define CHARACT_UUID  "110C0011-E8F2-537E-4F6C-D104768A1214"
#define BTN_NUM       47

LBLEClient client;
int serviceIdx = -1, charactIdx = -1;
bool bOn = false;

void setup() {
  //Initialize serial
  Serial.begin(115200);
  pinMode(BTN_NUM, INPUT_PULLUP);

  // Initialize BLE subsystem
  Serial.println("BLE begin");
  LBLE.begin();
  while (!LBLE.ready()) {
    delay(10);
  }
  Serial.println("BLE ready");

  // start scanning nearby advertisements
  LBLECentral.scan();
}

bool printDeviceInfo(int i) {
  bool rc = false;
  Serial.print(i); Serial.print("\t");
  Serial.print(LBLECentral.getAddress(i)); Serial.print("\t");
  Serial.print(LBLECentral.getAdvertisementFlag(i), HEX); Serial.print("\t");
  Serial.print(LBLECentral.getRSSI(i)); Serial.print("\t");

  const String name = LBLECentral.getName(i);
  Serial.print(name);
  if(name.length() == 0) Serial.print("(Unknown)");
  else if( name == TARGET_NAME ) rc = true;
  
  Serial.print(" by ");
  const String manu = LBLECentral.getManufacturer(i);
  Serial.print(manu);
  Serial.print(", service: ");
  if (!LBLECentral.getServiceUuid(i).isEmpty()) {
    Serial.print(LBLECentral.getServiceUuid(i));
  } else {
    Serial.print("(no service info)");
  }

  if (LBLECentral.isIBeacon(i)) {
    LBLEUuid uuid;
    uint16_t major = 0, minor = 0;
    int8_t txPower = 0;
    LBLECentral.getIBeaconInfo(i, uuid, major, minor, txPower);

    Serial.print(" ");
    Serial.print("iBeacon->");
    Serial.print("  UUID: "); Serial.print(uuid);
    Serial.print("\tMajor:"); Serial.print(major);
    Serial.print("\tMinor:"); Serial.print(minor);
    Serial.print("\ttxPower:"); Serial.print(txPower);
  }

  Serial.println();
  return rc;
}

int searching = 1;

enum AppState
{
  SEARCHING,    // We scan nearby devices and provide a list for user to choose from
  CONNECTING,   // User has choose the device to connect to
  CONNECTED     // We have connected to the device
};

void loop() {
  static AppState state = SEARCHING;
  static LBLEAddress serverAddress;

  switch(state)
  {
  case SEARCHING:
    {
      // wait for a while
      Serial.println("state=SEARCHING");
      for(int i = 0; i < 10; ++i)
      {
        delay(1000);
        Serial.print(".");
      }
      // enumerate advertisements found.
      Serial.print("Peripherals found = ");
      Serial.println(LBLECentral.getPeripheralCount());
      Serial.println("idx\taddress\t\t\tflag\tRSSI");
      for (int i = 0; i < LBLECentral.getPeripheralCount(); ++i) {
        if( printDeviceInfo(i) )
        {
          serverAddress = LBLECentral.getBLEAddress(i);
          Serial.print("Connect to device with address ");
          Serial.println(serverAddress);
          // we must stop scan before connecting to devices
          LBLECentral.stopScan();
          state = CONNECTING;
          break;
        }
      }
    }
    break;
  case CONNECTING:
  {
    Serial.println("state=CONNECTING");
    client.connect(serverAddress);
    
    if(client.connected()) state = CONNECTED;
    else Serial.println("can't connect");
  }
  break;

  case CONNECTED:
  {
    Serial.println("state=CONNECTED");

    // display all services of the remote device
    const int serviceCount = client.getServiceCount();
    Serial.println("available services = ");
    for(int i = 0; i < serviceCount; ++i)
    {
      Serial.print("\t - ");
      const String serviceName = client.getServiceName(i);
      Serial.print("["); 
      Serial.print(serviceName.length() > 0 ? serviceName : "NoName"); Serial.print("] ");
      Serial.println(client.getServiceUuid(i));

      Serial.println("\t\tCharacteristicUuid --");
      for(int k = 0; k < client.getCharacteristicCount(i); k++)
      {
        Serial.print("\t\t"); Serial.print(k); Serial.print(": "); 
        Serial.println( client.getCharacteristicUuid(i, k) );

        if( client.hasService( SERVICE_UUID ) && client.getCharacteristicUuid(i, k) == CHARACT_UUID )
        {
          serviceIdx = i;
          charactIdx = k;
          Serial.print("Service:"); Serial.print(serviceIdx); Serial.print(" , "); 
          Serial.print("Character:"); Serial.print(charactIdx); Serial.print("\r\n"); 
        }
      }
    }

    // enter idle state.
    while(true)
    {
      if( serviceIdx != -1 && charactIdx != -1 )
      {
        if( bOn != digitalRead(BTN_NUM) )
        {
          Serial.println(digitalRead(BTN_NUM));
          bOn = digitalRead(BTN_NUM) ? true : false;
          client.writeCharacteristicString(serviceIdx, charactIdx, bOn ? "1" : "0");
//        bOn = !bOn;
        }

        if( !client.connected() )
        {
          serviceIdx = charactIdx = -1;
          state = CONNECTING; // try to connect
          break;
        }
      }

      delay(100);
    }
  }
  break;
  }
}
