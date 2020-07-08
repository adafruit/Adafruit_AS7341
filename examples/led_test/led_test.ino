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
}

void loop() {

  // Function defined to read out channels with SMUX configration 1- F1-F4,
  // Clear, NIR
  as7341.readRawValuesMode1();



  // higher current
  // test led 
  // check polarity
  // text another breakout
  // check if in sleep
  // 4 = 12mA
  // 20 = 40+4 = 44mA
  Serial.println("Setting LED Current to 10/24mA");
//  as7341.setLEDCurrent(10); // 24mA
  as7341.setLEDCurrent(0); // 40mA
//  as7341.setLEDCurrent(48); // 100mA 
  Serial.println("Enabling LED");
  as7341.enableLED(true);
  delay(10);
  Serial.println("Disabling LED");
  as7341.enableLED(false);
  delay(2000);
  Serial.println("Done; end loop");
}
