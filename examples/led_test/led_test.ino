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

  Serial.println("Setup Atime, ASTEP, Gain");
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  delay(1000);
}

void loop() {

  delay(1000);
  as7341.setLEDCurrent(48); // 100mA
  delay(1000);
  as7341.enableLED(true);
  delay(10);
  as7341.enableLED(false);
  delay(500);
  as7341.setLEDCurrent(0); // 40mA
  as7341.enableLED(true);
  delay(200);
  as7341.enableLED(false);
  delay(100);
  as7341.setLEDCurrent(48); // 100mA
  as7341.enableLED(true);
  delay(10);
  as7341.enableLED(false);
  delay(500);
  as7341.enableLED(true);
  delay(200);
  as7341.enableLED(false);
  delay(100);

  as7341.enableLED(false);
}
