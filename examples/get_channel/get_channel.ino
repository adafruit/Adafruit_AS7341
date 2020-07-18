
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

  // read all 10 channels at the same time
  if (!as7341.readAllChannels()){
    Serial.println("Error reading all channels!");

    delay(100);
    return;
  }
  // Print out the stored values for each channel
  Serial.print("F1 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_415nm_F1));
  Serial.print("F2 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_445nm_F2));
  Serial.print("F3 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_480nm_F3));
  Serial.print("F4 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_515nm_F4));
  Serial.print("F5 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_555nm_F5));
  Serial.print("F6 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_590nm_F6));
  Serial.print("F7 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_630nm_F7));
  Serial.print("F8 - ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_680nm_F8));
  Serial.print("Clear - ");

  Serial.println(as7341.getChannel(AS7341_CHANNEL_CLEAR));
  Serial.print("Near IR - ");

  Serial.println(as7341.getChannel(AS7341_CHANNEL_NIR));
  Serial.println("");

  delay(2000);
}
