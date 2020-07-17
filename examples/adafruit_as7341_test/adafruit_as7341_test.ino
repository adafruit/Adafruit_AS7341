
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
    return;
  }

  // delay(1000);
  Serial.print("F1- "); Serial.print(readings[0], HEX); Serial.print(" ");Serial.println(readings[0]);
  Serial.print("F2- "); Serial.print(readings[1], HEX); Serial.print(" ");Serial.println(readings[1]);
  Serial.print("F3- "); Serial.print(readings[2], HEX); Serial.print(" ");Serial.println(readings[2]);
  Serial.print("F4- "); Serial.print(readings[3], HEX); Serial.print(" ");Serial.println(readings[3]);
  Serial.print("Clear - "); Serial.print(readings[4], HEX); Serial.print(" ");Serial.println(readings[4]);
  Serial.print("NIR - "); Serial.print(readings[5], HEX); Serial.print(" ");Serial.println(readings[5]);

  Serial.println("********************* ORIG **********");
  as7341.readRawValuesMode1();

  delay(500);
}
