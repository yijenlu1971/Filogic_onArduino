#include "Arduino.h"

void setup() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  Serial.println("off");
  delay(1000);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  Serial.println("on");
  delay(1000);
}
