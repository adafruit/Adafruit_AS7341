/* This example will fire an interrupt when a color channel goes beyond a set limit */

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

  Serial.println("**********  SETUP  *************");

  Serial.println("Disabling measurements");
  as7341.enableSpectralMeasurement(false);
  
  // Setup gain and sampling time
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_32X);
    
  as7341.setLowThreshold(250);
  as7341.setHighThreshold(10000);
  Serial.print("Low threshold set to ");
  Serial.println(as7341.getLowThreshold());
  Serial.print("High threshold set to "); 
  Serial.println(as7341.getHighThreshold());

  // How many cycles do we have to be outside thresh to
  // trigger an interrupt? If AS7341_INT_COUNT_ALL, every cycle
  as7341.setAPERS(AS7341_INT_COUNT_ALL);

  // Which channel to trigger on?
  as7341.setSpectralThresholdChannel(AS7341_ADC_CHANNEL_4);
  
  Serial.println("Enabling spectral interrupt");
  as7341.enableSpectralInterrupt(true);
  Serial.println("Enabling spectral measurements");
  as7341.enableSpectralMeasurement(true);
  Serial.println("*********************************");

  delay(1000);
}

void loop() {
  // Function defined to read out channels with SMUX configration 1- F1-F4, Clear, NIR
  as7341.readAllChannels();
  Serial.print("Clear    : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_CLEAR));
  
  uint8_t int_status = as7341.getInterruptStatus();
  if (int_status & 0x80) {
    Serial.println("Spectral interrupt");
  } else {
    // No IRQ!
    return;
  }

  uint8_t int_source = as7341.spectralInterruptSource();
  if (int_source & 0x10) {
    Serial.println("Thresh low interrupt");
  }
  if (int_source & 0x20) {
    Serial.println("Thresh high interrupt");
  }

  as7341.clearInterruptStatus();
}