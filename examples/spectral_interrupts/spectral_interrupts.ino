
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


  // DEFAULT IS 0, every cycle can trigger an interrupt
  // bool setAPERS(as7341_int_cycle_count_t cycle_count);

  // default channel is 0
  // bool setSpectralThresholdChannel(as7341_channel_t channel);

  Serial.println("**********  SETUP  *************");

  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_32X);
  Serial.println("Disabling measurements");
  as7341.enableSpectralMeasurement(false);
  Serial.println("Set low thresh");
  as7341.setLowThreshold(1);
  Serial.println("set high thresh");
  as7341.setHighThreshold(65000);

  Serial.print("Low threshold set to "); Serial.println((uint16_t)as7341.getLowThreshold());
  Serial.print("High threshold set to "); Serial.println((uint16_t)as7341.getHighThreshold());
  Serial.print("Enabling spectral ints");
  as7341.enableSpectralINT(true);
  Serial.println("Enabling spectral measurements");
  as7341.enableSpectralMeasurement(true);
  delay(3000);
  Serial.println("*********************************");
  // check default APERS and Channel

}

void loop() {
  // Function defined to read out channels with SMUX configration 1- F1-F4,
  // Clear, NIR
  as7341.readRawValuesMode1();
  uint8_t int_status = as7341.getInterruptStatus();
  uint8_t int_source = as7341.spectralINTSource();
  Serial.print("Thresh int source: "); Serial.println(int_source); 
  bool spectral_int_status = as7341.spectralInterruptTriggered();
  Serial.print("INT LOW THRESH: "); Serial.println(as7341.spectralLowTriggered());
  Serial.print("INT HIGH THRESH: "); Serial.println(as7341.spectralHighTriggered());
  as7341.clearInterruptStatus();
  Serial.print("INTs cleared? "); Serial.println(as7341.getInterruptStatus());
  Serial.print("Int status byte: 0b"); Serial.println(int_status, BIN);
  Serial.println("");
  Serial.print("spectral INT triggered: ");Serial.println(spectral_int_status);
  delay(2000);
}
