
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

  // bool setHighThreshold(int16_t high_threshold);
  // bool setLowThreshold(int16_t low_threshold);
  // bool enableSpectralINT(bool enable_int);
  // bool setAPERS(as7341_int_cycle_count_t cycle_count);
  // bool setSpectralThresholdChannel(as7341_channel_t channel);

  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

}

void loop() {
  // Function defined to read out channels with SMUX configration 1- F1-F4,
  // Clear, NIR
  as7341.readRawValuesMode1();


  delay(500);
}
