/* This example will set the onboard LED current to various values and blink
   the LED */

#include <Adafruit_AS7341.h>

Adafruit_AS7341 as7341;


void setup() {
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }

  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
}

void loop() {
  Serial.println("4 mA LED blink");
  as7341.setLEDCurrent(4); // 4mA
  as7341.enableLED(true);
  delay(100);
  as7341.enableLED(false);
  delay(500);

  Serial.println("100 mA LED blink");
  as7341.setLEDCurrent(100); // 100mA
  as7341.enableLED(true);
  delay(100);
  as7341.enableLED(false);
  delay(500);
}
