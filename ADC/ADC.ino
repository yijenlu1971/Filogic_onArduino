#define ADC0_PIN    A0
#define ADC1_PIN    A1
#define ADC2_PIN    A4
#define ADC3_PIN    A5

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t val = analogRead(ADC0_PIN);
  Serial.print(val); Serial.print(',');
  
  val = analogRead(ADC1_PIN);
  Serial.print(val); Serial.print(',');
  val = analogRead(ADC2_PIN);
  Serial.print(val); Serial.print(',');
  val = analogRead(ADC3_PIN);
  Serial.print(val); Serial.print(',');

  val = analogRead(HAL_GPIO_27); // ADC-10
  Serial.print(val); Serial.print(',');

  val = analogRead(HAL_GPIO_28); // ADC-11
  Serial.println(val);
  delay(500);
}
