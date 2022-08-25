void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t val = analogRead(HAL_GPIO_27);
  Serial.print(val);
  Serial.print(',');

  val = analogRead(HAL_GPIO_28);
  Serial.println(val);
  delay(500);
}
