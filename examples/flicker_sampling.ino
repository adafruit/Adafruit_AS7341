/** This example will read all channels from the AS7341.
  * The raw ADC values will then be converted to basic counts
  * according to the sensor's appliation note:
  * 
  * https://ams.com/documents/20143/36005/AS7341_AN000633_1-00.pdf/fc552673-9800-8d60-372d-fc67cf075740
  * Section 3.1
  * 
  * BasicCounts = RawSensorValue / ( Gain x IntegrationTime )
  * 
  * Finally, the converted values are printed out.
  */
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

int16_t led_current_ma = 258;

void loop() {
  //======= FLICKER =========
  uint16_t wbuf[16];
  uint16_t wlen = 16;
  uint16_t freq_hz = 1200;
  uint8_t fd_gain = AS7341_GAIN_512X; 
  uint16_t fd_time = as7341.sampleFicker(wbuf, wlen, freq_hz, fd_gain);
  
  //print results
  Serial.print("FLICKER  ");   
  for(uint16_t i=0; i<wlen; i++) {
    Serial.print(wbuf[i]);
    Serial.print(" ");
  }
  Serial.print("max="); 
  Serial.print(fd_time);
  Serial.println();

  //======= SPECTRUM =========   
  uint16_t counts[12];
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  
  if (!as7341.readAllChannels(counts)){
    Serial.println("Error reading all channels!");
  }else{
    Serial.print("SPECTRUM ");
    Serial.print(counts[0]);
    Serial.print(" ");
    Serial.print(counts[1]);
    Serial.print(" ");
    Serial.print(counts[2]);
    Serial.print(" ");
    Serial.print(counts[3]);
    // again, we skip the duplicates  
    Serial.print(" ");
    Serial.print(counts[6]);
    Serial.print(" ");
    Serial.print(counts[7]);
    Serial.print(" ");
    Serial.print(counts[8]);
    Serial.print(" ");
    Serial.print(counts[9]);
    Serial.print(" Clear=");
    Serial.print(counts[10]);
    Serial.print(" NIR=");
    Serial.print(counts[11]);
    Serial.println();
  }

  delay(500);
}
