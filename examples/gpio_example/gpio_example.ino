/* This example will use the GPIO pin to blink an led */

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
  // set the GPIO direction to output so we can set it high or low
  as7341.setGPIODirection(AS7341_GPIO_OUTPUT);
}

void loop() {
  Serial.println("Setting GPIO High");
  as7341.setGPIOValue(true);
  delay(3000);
  Serial.println("Setting GPIO Low");
  as7341.setGPIOValue(false);
  delay(500);
}
