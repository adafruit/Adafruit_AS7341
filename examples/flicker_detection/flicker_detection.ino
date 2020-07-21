/* This example will check for light flicker effects. Household incandescent,
   fluorescent and LED lights may have flickering */


#include <Adafruit_AS7341.h>

Adafruit_AS7341 as7341;


void setup() {
  // Wait for communication with the host computer serial monitor
  Serial.begin(115200);

  while (!Serial) {
    delay(1);
  }

  if (!as7341.begin()){
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }

  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
}

void loop() {
  uint16_t flicker_freq = 0;

  flicker_freq = as7341.detectFlickerHz();

  if (flicker_freq == 0) {
      Serial.println("No flicker detected");
  }
  else if (flicker_freq == 1) {
      Serial.println("Flicker detected with unknown frequency");
  } 
  else {
      Serial.print("Flicker detected at ");
      Serial.print(flicker_freq);
      Serial.println(" Hz");
  }
}

