// Basic demo for accelerometer readings from Adafruit AS7341

Adafruit_AS7341 as7341;


}



// Basic demo for readings from Adafruit AS7341
#include <Wire.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_Sensor.h>

// For SPI mode, we need a CS pin
#define AS7341_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define AS7341_SCK 13
#define AS7341_MISO 12
#define AS7341_MOSI 11

Adafruit_AS7341  as7341; // TODO FIX NAME
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit AS7341 test!");

  // Try to initialize!
  if (!as7341.begin_I2C()) {
  //if (!as7341.begin_SPI(AS7341_CS)) {
  //if (!as7341.begin_SPI(AS7341_CS, AS7341_SCK, AS7341_MISO, AS7341_MOSI)) {
    Serial.println("Failed to find AS7341 chip");
    while (1) { delay(10); }
  }
  Serial.println("AS7341 Found!");

//  as7341.setDataRate(AS7341_RATE_12_5_HZ);
  Serial.print("Data rate set to: ");
  switch (as7341.getDataRate()) {
    case AS7341_RATE_ONE_SHOT: Serial.println("One Shot"); break;
    case AS7341_RATE_1_HZ: Serial.println("1 Hz"); break;
    case AS7341_RATE_7_HZ: Serial.println("7 Hz"); break;

  }

void loop() {

  sensors_event_t temp;
  sensors_event_t pressure;
  as7341.getEvent(&pressure, &temp);// get pressure
  Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.println("");
  delay(100);
}
 sensors_event_t temp;
  sensors_event_t pressure;
  as7341.getEvent(&pressure, &temp);// get pressure
  Serial.print(" Temperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
  Serial.print(" Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.println("");
  delay(100);
}
