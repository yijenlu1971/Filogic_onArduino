/*
  This example connects to an unencrypted Wifi network.
  Then it prints the  MAC address of the Wifi shield,
  the IP address obtained, and other network details.

  Circuit:
  * Filogic 130 HDK

  created 13 July 2010
  by dlf (Metodo2 srl)
  modified 31 May 2012
  by Tom Igoe
  modified 23 May 2017
  by MediaTek Labs
*/
#include <LWiFi.h>
#include <LRTC.h>
#include <TLSClient.h>
#include "sntp.h"

#define LED_BUILTIN   7

char ssid[] = "ASUS";     //  your network SSID (name)
char pass[] = "0915058267";  // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
//TLSClient iotClient;
bool bOn = false;

extern u32_t sntp_timestamp;

void setup() {
  IPAddress ntpAddr, dns(8, 8, 8, 8);
  char  *hostname = "time.stdtime.gov.tw";
  char  *get_url = "https://api.thingspeak.com/update";
  int   tryCnt = 0;
  struct tm  *tmInfo;
  time_t ut;

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

	// attempt to connect to Wifi network:
	while (status != WL_CONNECTED) {
		Serial.print("Attempting to connect to WPA SSID: ");
		Serial.println(ssid);
		// Connect to WPA/WPA2 network:
		status = WiFi.begin(ssid, pass);
	}

	// you're connected now, so print out the data:
	Serial.print("You're connected to the network");
	printCurrentNet();
	printWifiData();

  // start the RTC module
  LRTC.begin();

  WiFi.setDNS(dns);
  if( WiFi.hostByName(hostname, ntpAddr) )
  {
    Serial.print("NTP IP: "); Serial.println(ntpAddr);
    do {
      sntp_init(ntpAddr);
      delay(1000);
      if( ++tryCnt >= 5 ) break;
    } while ( sntp_recvfrom() == 0 );

    ut = time(NULL) + sntp_timestamp + (8*60*60);
    tmInfo = localtime(&ut);
    LRTC.set(tmInfo->tm_year+1900, tmInfo->tm_mon+1, tmInfo->tm_mday, 
        tmInfo->tm_hour, tmInfo->tm_min, tmInfo->tm_sec);
  }
}

void loop() {
  char buffer[64];

	// check the network connection once every 10 seconds:
	delay(10000);
  // get time from the RTC module
  LRTC.get();

  // display the time
  sprintf(buffer, "%ld/%ld/%ld %.2ld:%.2ld:%.2ld",
    LRTC.year(), LRTC.month(), LRTC.day(), LRTC.hour(), LRTC.minute(), LRTC.second());
  Serial.println(buffer);

  digitalWrite(LED_BUILTIN, bOn ? HIGH : LOW);
  bOn = !bOn;
}

void printWifiData() {
	// print your WiFi shield's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: "); Serial.println(ip);
	//Serial.print("Gateway: "); Serial.println(WiFi.gatewayIP());

	// print your MAC address:
	byte mac[6];
	WiFi.macAddress(mac);
	Serial.print("MAC address: ");
	Serial.print(mac[5], HEX); Serial.print(":");
	Serial.print(mac[4], HEX); Serial.print(":");
	Serial.print(mac[3], HEX); Serial.print(":");
	Serial.print(mac[2], HEX); Serial.print(":");
	Serial.print(mac[1], HEX); Serial.print(":");
	Serial.println(mac[0], HEX);
}

void printCurrentNet() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: "); Serial.println(WiFi.SSID());

	// print the MAC address of the router you're attached to:
	byte bssid[6];
	WiFi.BSSID(bssid);
	Serial.print("BSSID: ");
	Serial.print(bssid[5], HEX); Serial.print(":");
	Serial.print(bssid[4], HEX); Serial.print(":");
	Serial.print(bssid[3], HEX); Serial.print(":");
	Serial.print(bssid[2], HEX); Serial.print(":");
	Serial.print(bssid[1], HEX); Serial.print(":");
	Serial.println(bssid[0], HEX);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):"); Serial.print(rssi); Serial.println(" dBm");

	// print the encryption type:
	byte encryption = WiFi.encryptionType();
	Serial.print("Encryption Type:");
	Serial.println(encryption, HEX);
	printEncryptionType(encryption);
	Serial.println();
}
void printEncryptionType(int thisType) {
  // read the encryption type and print out the name:
  switch (thisType) {
    case ENC_TYPE_WEP:
      Serial.println("WEP");
      break;
    case ENC_TYPE_TKIP:
      Serial.println("WPA");
      break;
    case ENC_TYPE_CCMP:
      Serial.println("WPA2");
      break;
    case ENC_TYPE_NONE:
      Serial.println("None");
      break;
    case ENC_TYPE_AUTO:
      Serial.println("Auto");
      break;
    default :
      Serial.println("WPA-PSK/WPA2-PSK");
      break;
  }
}
