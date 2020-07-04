
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
}

void loop() {


  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);

  // Function defined to read out channels with SMUX configration 1- F1-F4,
  // Clear, NIR
  as7341.readRawValuesMode1();

  // Function defined to read out channels with SMUX configration 2- F5-F8,
  // Clear, NIR
  as7341.readRawValuesMode2();
  delay(1000);

  //  // Function detects Flicker for 100 and 120 Hz
  //  flickerDetection();

  as7341.flickerDetection1K();

  // reading the flicker status in FD_STATUS register 0xDB
  // data=0x2c=0b00101100  FD_STATUS(fd_measurement_valid=1
  // fd_120Hz_flicker_valid=1 fd_100Hz_flicker_valid=1) data=0x2d=0b00101101
  // FD_STATUS(fd_measurement_valid=1 fd_1200Hz_flicker_valid=1
  // fd_1000Hz_flicker_valid=1 fd_1000Hz_flicker) data=0x2e=0b00101110
  // FD_STATUS(fd_measurement_valid=1 fd_1200Hz_flicker_valid=1
  // fd_1000Hz_flicker_valid=1 fd_1200Hz_flicker)

  int flicker_value = as7341.getFlickerValue();
               Serial.print("Flicker Status-");
               Serial.println(flicker_value);


  Serial.println("** 1k flicker test **");
  if (flicker_value == 44) {
    Serial.println("Unknown frequency");
  } else if (flicker_value == 45) {
    Serial.println("1000 Hz detected");
  } else if (flicker_value == 46) {
    Serial.println("1200 Hz detected");
  } else {
    Serial.println("Error in reading");
  }

  Serial.println("** 1k flicker test end **");
  delay(500);
}
