
#include <Adafruit_AS7341.h>
#include <Wire.h>
Adafruit_AS7341 as7341;
void setup() {
  // communication with the host computer serial monitor
  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
}

void loop() {
  uint16_t readings[6];


  if (!as7341.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    // return;
  }
  Serial.print("ADC0/F1-");
  Serial.println(readings[0]);
  Serial.print("ADC1/F2-");
  Serial.println(readings[1]);
  Serial.print("ADC2/F3-");
  Serial.println(readings[2]);
  Serial.print("ADC3/F4-");
  Serial.println(readings[3]);
  Serial.print("ADC4/Clear-");
  Serial.println(readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(readings[5]);
  Serial.print("ADC0/F5-");
  Serial.println(readings[6]);
  Serial.print("ADC1/F6-");
  Serial.println(readings[7]);
  Serial.print("ADC2/F7-");
  Serial.println(readings[8]);
  Serial.print("ADC3/F8-");
  Serial.println(readings[9]);
  Serial.print("ADC4/Clear-");
  Serial.println(readings[10]);
  Serial.print("ADC5/NIR-");
  Serial.println(readings[11]);

  delay(2000);
}
