/*!
 *  @file Adafruit_AS7341.cpp
 *
 *  @mainpage Adafruit AS7341 11-Channel Spectral Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the AS7341 11-Channel Spectral Sensor
 *
 * 	This is a library for the Adafruit AS7341 breakout:
 * 	https://www.adafruit.com/product/45XX
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  This library depends on the Adafruit Unified Sensor library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_AS7341.h"

/**
 * @brief Construct a new Adafruit_AS7341::Adafruit_AS7341 object
 *
 */
Adafruit_AS7341::Adafruit_AS7341(void) {}

/**
 * @brief Destroy the Adafruit_AS7341::Adafruit_AS7341 object
 *
 */
Adafruit_AS7341::~Adafruit_AS7341(void) {
  //   if (temp_sensor)
  //     delete temp_sensor;
  //   if (pressure_sensor)
  //     delete pressure_sensor;
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_AS7341::begin(uint8_t i2c_address, TwoWire *wire,
                            int32_t sensor_id) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  return _init(sensor_id);
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_AS7341::_init(int32_t sensor_id) {

  Adafruit_BusIO_Register chip_id =
      Adafruit_BusIO_Register(i2c_dev, AS7341_WHOAMI);

  // make sure we're talking to the right chip
  if (chip_id.read() & 0xFC != AS7341_CHIP_ID << 2) {
    return false;
  }

  powerEnable(true);
  return true;
}

/********************* EXAMPLE EXTRACTS **************/
// maybe return a typedef enum
int8_t Adafruit_AS7341::getFlickerValue(void) {
  // int flicker_value = as7341.readRegister(byte(AS7341_FD_STATUS));
  Adafruit_BusIO_Register flicker_val =
      Adafruit_BusIO_Register(i2c_dev, AS7341_FD_STATUS);
  return (int8_t)flicker_val.read();
}

// <summary>
// Read two consecutive i2c registers
// <summary>
// param name = "addr">First register address of two consecutive registers to be
// read param name = "AS7341_I2CADDR_DEFAULT">Device address 0x39

uint16_t Adafruit_AS7341::readTwoRegister1(byte addr) {
  uint8_t readingL;
  uint16_t readingH;
  uint16_t reading = 0;
  Wire.beginTransmission(AS7341_I2CADDR_DEFAULT);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(AS7341_I2CADDR_DEFAULT, 2);

  if (2 <= Wire.available()) {
    readingL = Wire.read();
    readingH = Wire.read();
    readingH = readingH << 8;
    reading = (readingH | readingL);
    return (reading);
  } else {
    Serial.println("I2C Error");
    return (0xFFFF); // Error
  }
}

// <summary>
// Write a value to a single i2c register
// <summary>
// param name = "addr">Register address of the the register to the value to be
// written param name = "val">The value written to the Register param name =
// "AS7341_I2CADDR_DEFAULT">Device address 0x39

void Adafruit_AS7341::writeRegister(byte addr, byte val) {
  Wire.beginTransmission(AS7341_I2CADDR_DEFAULT);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
}

/*----- Register configuration  -----*/

// <summary>
// Setting the PON (Power on) bit on the chip (bit0 at register ENABLE 0x80)
// Attention: This function clears only the PON bit in ENABLE register and keeps
// the other bits <summary>
void Adafruit_AS7341::powerEnable(bool enable_power) {
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
  Adafruit_BusIO_RegisterBits pon_en =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 0);
  pon_en.write(enable_power);
}

// <summary>
// Write SMUX configration from RAM to set SMUX chain in CFG6 register 0xAF
// <summary>

void Adafruit_AS7341::SmuxConfigRAM() { writeRegister(byte(0xAF), byte(0x10)); }

bool Adafruit_AS7341::enableSpectralMeasurement(bool enable_measurement) {

  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);

  Adafruit_BusIO_RegisterBits spec_enable_bit =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 1);
  return spec_enable_bit.write(enable_measurement);
}

void Adafruit_AS7341::enableSMUX(void) {
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
  Adafruit_BusIO_RegisterBits smux_enable_bit =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 4);
  smux_enable_bit.write(true);
  while (smux_enable_bit.read()) {
    delay(1);
  }
}
// <summary>
// Reading and Polling the the AVALID bit in Status 2 Register 0xA3,if the
// spectral measurement is ready or busy. True indicates that a cycle is
// completed since the last readout of the Raw Data register <summary>

bool Adafruit_AS7341::getIsDataReady() {
  bool isDataReady = false;
  byte regVal = readRegister(byte(AS7341_STATUS2));

  if ((regVal & 0x40) == 0x40) {

    return isDataReady = true;
  }

  else {
    return isDataReady = false;
  }
}

//<summary>
// Reading and polling of Flicker measurement ready bit (bit [5] on FD_Status
// register True indicates that the Flicker Detection measurement was finished
//<summary>

bool Adafruit_AS7341::getFdMeasReady() {
  bool isFdmeasReady = false;
  byte regVal = readRegister(byte(AS7341_FD_STATUS));

  if ((regVal & 0x20) == 0x20) {

    return isFdmeasReady = true;
  }

  else {
    return isFdmeasReady = false;
  }
}

/*----- SMUX Configuration for F1,F2,F3,F4,CLEAR,NIR -----*/

//<summary>
// Mapping the individual Photo diodes to dedicated ADCs using SMUX
// Configuration for F1-F4,Clear,NIR
//<summary>

void Adafruit_AS7341::F1F4_Clear_NIR() {
  // SMUX Config for F1,F2,F3,F4,NIR,Clear
  writeRegister(byte(0x00), byte(0x30)); // F3 left set to ADC2
  writeRegister(byte(0x01), byte(0x01)); // F1 left set to ADC0
  writeRegister(byte(0x02), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x03), byte(0x00)); // F8 left disabled
  writeRegister(byte(0x04), byte(0x00)); // F6 left disabled
  writeRegister(
      byte(0x05),
      byte(0x42)); // F4 left connected to ADC3/f2 left connected to ADC1
  writeRegister(byte(0x06), byte(0x00)); // F5 left disbled
  writeRegister(byte(0x07), byte(0x00)); // F7 left disbled
  writeRegister(byte(0x08), byte(0x50)); // CLEAR connected to ADC4
  writeRegister(byte(0x09), byte(0x00)); // F5 right disabled
  writeRegister(byte(0x0A), byte(0x00)); // F7 right disabled
  writeRegister(byte(0x0B), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x0C), byte(0x20)); // F2 right connected to ADC1
  writeRegister(byte(0x0D), byte(0x04)); // F4 right connected to ADC3
  writeRegister(byte(0x0E), byte(0x00)); // F6/F7 right disabled
  writeRegister(byte(0x0F), byte(0x30)); // F3 right connected to AD2
  writeRegister(byte(0x10), byte(0x01)); // F1 right connected to AD0
  writeRegister(byte(0x11), byte(0x50)); // CLEAR right connected to AD4
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x06)); // NIR connected to ADC5
}

/*----- SMUX Configuration for F5,F6,F7,F8,CLEAR,NIR -----*/

//<summary>
// Mapping the individual Photo diodes to dedicated ADCs using SMUX
// Configuration for F5-F8,Clear,NIR
//<summary>

void Adafruit_AS7341::F5F8_Clear_NIR() {
  // SMUX Config for F5,F6,F7,F8,NIR,Clear
  writeRegister(byte(0x00), byte(0x00)); // F3 left disable
  writeRegister(byte(0x01), byte(0x00)); // F1 left disable
  writeRegister(byte(0x02), byte(0x00)); // reserved/disable
  writeRegister(byte(0x03), byte(0x40)); // F8 left connected to ADC3
  writeRegister(byte(0x04), byte(0x02)); // F6 left connected to ADC1
  writeRegister(byte(0x05), byte(0x00)); // F4/ F2 disabled
  writeRegister(byte(0x06), byte(0x10)); // F5 left connected to ADC0
  writeRegister(byte(0x07), byte(0x03)); // F7 left connected to ADC2
  writeRegister(byte(0x08), byte(0x50)); // CLEAR Connected to ADC4
  writeRegister(byte(0x09), byte(0x10)); // F5 right connected to ADC0
  writeRegister(byte(0x0A), byte(0x03)); // F7 right connected to ADC2
  writeRegister(byte(0x0B), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x0C), byte(0x00)); // F2 right disabled
  writeRegister(byte(0x0D), byte(0x00)); // F4 right disabled
  writeRegister(byte(0x0E),
                byte(0x24)); // F7 connected to ADC2/ F6 connected to ADC1
  writeRegister(byte(0x0F), byte(0x00)); // F3 right disabled
  writeRegister(byte(0x10), byte(0x00)); // F1 right disabled
  writeRegister(byte(0x11), byte(0x50)); // CLEAR right connected to AD4
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x06)); // NIR connected to ADC5
}

/*----- //SMUX Configuration for Flicker detection - register (0x13)left set to
 * ADC6 for flicker detection-----*/

//<summary>
// Mapping the individual Photo diodes to dedicated ADCs using SMUX
// Configuration for Flicker detection
//<summary>

void Adafruit_AS7341::FDConfig() {
  // SMUX Config for Flicker- register (0x13)left set to ADC6 for flicker
  // detection
  writeRegister(byte(0x00), byte(0x00)); // disabled
  writeRegister(byte(0x01), byte(0x00)); // disabled
  writeRegister(byte(0x02), byte(0x00)); // reserved/disabled
  writeRegister(byte(0x03), byte(0x00)); // disabled
  writeRegister(byte(0x04), byte(0x00)); // disabled
  writeRegister(byte(0x05), byte(0x00)); // disabled
  writeRegister(byte(0x06), byte(0x00)); // disabled
  writeRegister(byte(0x07), byte(0x00)); // disabled
  writeRegister(byte(0x08), byte(0x00)); // disabled
  writeRegister(byte(0x09), byte(0x00)); // disabled
  writeRegister(byte(0x0A), byte(0x00)); // disabled
  writeRegister(byte(0x0B), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x0C), byte(0x00)); // disabled
  writeRegister(byte(0x0D), byte(0x00)); // disabled
  writeRegister(byte(0x0E), byte(0x00)); // disabled
  writeRegister(byte(0x0F), byte(0x00)); // disabled
  writeRegister(byte(0x10), byte(0x00)); // disabled
  writeRegister(byte(0x11), byte(0x00)); // disabled
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13),
                byte(0x60)); // Flicker connected to ADC5 to left of 0x13
}

/*----- Set integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS -----*/

//<summary>
// Sets the ATIME for integration time from 0 to 255, integration time = (ATIME
// + 1) * (ASTEP + 1) * 2.78µS
//<summary>
// param name = "value"> integer value from 0 to 255 written to ATIME register
// 0x81

// TODO; check for valid values
void Adafruit_AS7341::setATIME(byte value) {
  Adafruit_BusIO_Register atime_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ATIME);
  atime_reg.write(value);
}

//<summary>
// Sets the ASTEP for integration time from 0 to 65535, integration time =
// (ATIME + 1) * (ASTEP + 1) * 2.78µS
//<summary>
// param name = "value1,"> Defines the lower byte[7:0] of the base step time
// written to ASTEP register 0xCA param name = "value2,"> Defines the higher
// byte[15:8] of the base step time written to ASTEP register 0xCB

void Adafruit_AS7341::setASTEP(uint16_t astep_value) {
  Adafruit_BusIO_Register astep_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ASTEP_L, 2, LSBFIRST);
  astep_reg.write(astep_value);
}
//<summary>
// Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
//<summary>
// param name = "value"> integer value from 0 to 10 written to AGAIN register
// 0xAA

void Adafruit_AS7341::setGAIN(byte value) {
  writeRegister(byte(AS7341_CFG1), value);
}

/*----- Function to detect Flickering at 100 and 120 Hz(default detection in
 * XWing Sensor) -----*/

//<summary>
// Executing a flicker measurement cycle, displaying the status from FD_Status
// register
//<summary>

void Adafruit_AS7341::flickerDetection() {
  bool isEnabled = true;
  bool isFdmeasReady = false;

  // disable everything; Flicker detect, smux, wait, spectral, power
  writeRegister(byte(AS7341_ENABLE), byte(0x00));

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)
  SmuxConfigRAM();

  // Write new configuration to all the 20 registers for detecting Flicker
  FDConfig();

  // Start SMUX command
  enableSMUX();

  // Enable SP_EN bit
  enableSpectralMeasurement(true);

  /*----- Functions for setting Flicker Sample, Flicker time, Flicker Gain (not
   * implemented for default flicker detection)------*/
  //            writeRegister(byte(AS7341_FD_CFG0), byte(0x21)); //33 default
  //            value, function for setting for Fd_sample and Fd_compare_value
  //
  //            writeRegister(byte(AS7341_FD_TIME1), byte(0x68)); //104 default
  //            value, function for setting for Fd_time lower bit(7:0)
  //
  //            writeRegister(byte(AS7341_FD_TIME2), byte(0x49)); //73 default
  //            value, function for setting for fd_gain and fd_time higher
  //            bit(10:8)

  // Function to set the Flicker detection via enabling the fden bit in 0x80
  // register
  // enablePower(true);
  // enableFlickerDetect(true);
  writeRegister(byte(AS7341_ENABLE), byte(0x41));
  delay(500);

  // reading the flicker status in FD_STATUS register 0xDB

  int flicker_value = readRegister(byte(AS7341_FD_STATUS));
  Serial.print("Flicker value-");
  Serial.println(flicker_value);

  if (flicker_value == 44) {
    Serial.println("Unknown frequency");
  } else if (flicker_value == 45) {
    Serial.println("100 Hz detected");
  } else if (flicker_value == 46) {
    Serial.println("120 Hz detected");
  } else {
    Serial.println("Error in reading");
  }

  // Setting back the PON bit in the ENABLE Register

  writeRegister(byte(AS7341_ENABLE), byte(0x01));

  Serial.println("");
}

/*##################### 1K Flicker detect
 * #######################################*/

// <summary>
// To detect the target frequency of 1kHz and further 1.2kHz Flickering
// <summary>
void Adafruit_AS7341::flickerDetection1K() {

  // RAM_BANK 0 select which RAM bank to access in register addresses 0x00-0x7f
  writeRegister(byte(AS7341_CFG0), byte(0x00));

  // The coefficient calculated are stored into the RAM bank 0 and RAM bank 1,
  // they are used instead of 100Hz and 120Hz coefficients which are the default
  // flicker detection coefficients
  // write new coefficients to detect the 1000Hz and 1200Hz - part 1
  writeRegister(byte(0x04), byte(0x9E));
  writeRegister(byte(0x05), byte(0x36));
  writeRegister(byte(0x0E), byte(0x2E));
  writeRegister(byte(0x0F), byte(0x1B));
  writeRegister(byte(0x18), byte(0x7D));
  writeRegister(byte(0x19), byte(0x36));
  writeRegister(byte(0x22), byte(0x09));
  writeRegister(byte(0x23), byte(0x1B));
  writeRegister(byte(0x2C), byte(0x5B));
  writeRegister(byte(0x2D), byte(0x36));
  writeRegister(byte(0x36), byte(0xE5));
  writeRegister(byte(0x37), byte(0x1A));
  writeRegister(byte(0x40), byte(0x3A));
  writeRegister(byte(0x41), byte(0x36));
  writeRegister(byte(0x4A), byte(0xC1));
  writeRegister(byte(0x4B), byte(0x1A));
  writeRegister(byte(0x54), byte(0x18));
  writeRegister(byte(0x55), byte(0x36));
  writeRegister(byte(0x5E), byte(0x9C));
  writeRegister(byte(0x5F), byte(0x1A));
  writeRegister(byte(0x68), byte(0xF6));
  writeRegister(byte(0x69), byte(0x35));
  writeRegister(byte(0x72), byte(0x78));
  writeRegister(byte(0x73), byte(0x1A));
  writeRegister(byte(0x7C), byte(0x4D));
  writeRegister(byte(0x7D), byte(0x35));

  // RAM_BANK 1 select which RAM bank to access in register addresses 0x00-0x7f
  writeRegister(byte(AS7341_CFG0), byte(0x01));

  // write new coefficients to detect the 1000Hz and 1200Hz - part 1
  writeRegister(byte(0x06), byte(0x54));
  writeRegister(byte(0x07), byte(0x1A));
  writeRegister(byte(0x10), byte(0xB3));
  writeRegister(byte(0x11), byte(0x35));
  writeRegister(byte(0x1A), byte(0x2F));
  writeRegister(byte(0x1B), byte(0x1A));

  writeRegister(byte(AS7341_CFG0), byte(0x01));

  // select RAM coefficients for flicker detection by setting
  // fd_disable_constant_init to „1“ (FD_CFG0 register) in FD_CFG0 register -
  // 0xd7  fd_disable_constant_init=1 fd_samples=4
  writeRegister(byte(AS7341_FD_CFG0), byte(0x60));
  // readRegisterPrint(byte(AS7341_FD_CFG0));

  // in FD_CFG1 register - 0xd8 fd_time(7:0) = 0x40
  writeRegister(byte(AS7341_FD_TIME1), byte(0x40));
  // readRegisterPrint(byte(AS7341_FD_TIME1));

  // in FD_CFG2 register - 0xd9  fd_dcr_filter_size=1 fd_nr_data_sets(2:0)=5
  writeRegister(byte(0xD9), byte(0x25));
  // readRegisterPrint(byte(0xD9));

  // in FD_CFG3 register - 0xda fd_gain=9
  writeRegister(byte(AS7341_FD_TIME2), byte(0x48));
  // readRegisterPrint(byte(AS7341_FD_TIME2));

  // in CFG9 register - 0xb2 sien_fd=1
  writeRegister(byte(AS7341_CFG9), byte(0x40));
  // readRegisterPrint(byte(AS7341_CFG9));

  // in ENABLE - 0x80  fden=1 and pon=1 are enabled
  writeRegister(byte(AS7341_ENABLE), byte(0x41));
  // readRegisterPrint(byte(AS7341_ENABLE));
}

/*#####################  END 1K Flicker detect
 * #######################################*/

/*----- Function defined to read out channels with SMUX configration 1 -----*/

//<summary>
// Executing raw data measurement cycle for 6 channels F1,F2,F3,F4,NIR,Clear
//<summary>

void Adafruit_AS7341::readRawValuesMode1() {

  bool isEnabled = true;
  bool isDataReady = false;

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)

  SmuxConfigRAM();

  // Write new configuration to all the 20 registers

  F1F4_Clear_NIR();

  // Start SMUX command
  enableSMUX();

  // Enable SP_EN bit

  enableSpectralMeasurement(true);

  // Reading and Polling the the AVALID bit in Status 2 Register 0xA3

  while (!(isDataReady)) {
    isDataReady = getIsDataReady();
  }

  // Steps defined to print out 6 channels F1,F2,F3,F4,NIR,Clear

  Serial.print("ADC0/F1-");
  Serial.println(readTwoRegister1(0x95));
  Serial.print("ADC1/F2-");
  Serial.println(readTwoRegister1(0x97));
  Serial.print("ADC2/F3-");
  Serial.println(readTwoRegister1(0x99));
  Serial.print("ADC3/F4-");
  Serial.println(readTwoRegister1(0x9B));
  Serial.print("ADC4/Clear-");
  Serial.println(readTwoRegister1(0x9D));
  Serial.print("ADC5/NIR-");
  Serial.println(readTwoRegister1(0x9F));
}

/*----- Function defined to read out channels with SMUX configration 2 -----*/

//<summary>
// Executing raw data measurement cycle for 6 channels F1,F2,F3,F4,NIR,Clear
//<summary>

void Adafruit_AS7341::readRawValuesMode2() {
  bool isEnabled = true;
  bool isDataReady = false;
  ;

  // Disable SP_EN bit while  making config changes
  enableSpectralMeasurement(false);

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)
  SmuxConfigRAM();

  // Write new configuration to all the 20 registers for reading channels from
  // F5-F8, Clear and NIR
  F5F8_Clear_NIR();
  // Start SMUX command
  enableSMUX();

  // Enable SP_EN bit
  enableSpectralMeasurement(true);

  // Reading and Polling the the AVALID bit in Status 2 Register 0xA3

  while (!(isDataReady)) {
    isDataReady = getIsDataReady();
  }

  // Steps defined to printout 6 channels F5,F6,F7,F8,NIR,Clear

  Serial.print("ADC0/F5-");
  Serial.println(readTwoRegister1(0x95));
  Serial.print("ADC1/F6-");
  Serial.println(readTwoRegister1(0x97));
  Serial.print("ADC2/F7-");
  Serial.println(readTwoRegister1(0x99));
  Serial.print("ADC3/F8-");
  Serial.println(readTwoRegister1(0x9B));
  Serial.print("ADC4/Clear-");
  Serial.println(readTwoRegister1(0x9D));
  Serial.print("ADC5/NIR-");
  Serial.println(readTwoRegister1(0x9F));
  Serial.println("");
}

void Adafruit_AS7341::readRegisterPrint(byte addr) {
  Wire.beginTransmission(AS7341_I2CADDR_DEFAULT);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(AS7341_I2CADDR_DEFAULT, 1);

  if (Wire.available()) {
    Serial.println(Wire.read());
    // return (Wire.read());
  }

  else {
    Serial.println("I2C Error");
    // return (0xFF); //Error
  }
}
/* ----- Read/Write to i2c register ----- */

// <summary>
// Read a single i2c register
// <summary>
// param name = "addr">Register address of the the register to be read
// param name = "AS7341_I2CADDR_DEFAULT">Device address 0x39

byte Adafruit_AS7341::readRegister(byte addr) {
  Wire.beginTransmission(AS7341_I2CADDR_DEFAULT);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(AS7341_I2CADDR_DEFAULT, 1);

  if (Wire.available()) {
    // Serial.println(Wire.read());
    return (Wire.read());
  }

  else {
    Serial.println("I2C Error");
    return (0xFF); // Error
  }
}

// /**
//  * @brief Gets the current rate at which pressure and temperature
//  measurements
//  * are taken
//  *
//  * @return as7341_rate_t The current data rate
//  */
// as7341_rate_t Adafruit_AS7341::getDataRate(void) {
//   Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AS7341_CTRL_REG1, 1);
//   Adafruit_BusIO_RegisterBits data_rate =
//       Adafruit_BusIO_RegisterBits(&ctrl1, 3, 4);

//   return (as7341_rate_t)data_rate.read();
// }
// /**
//  * @brief Sets the rate at which pressure and temperature measurements
//  *
//  * @param new_data_rate The data rate to set. Must be a `as7341_rate_t`
//  */
// void Adafruit_AS7341::Adafruit_AS7341::setDataRate(as7341_rate_t
// new_data_rate) {
//   Adafruit_BusIO_Register ctrl1 = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AS7341_CTRL_REG1, 1);
//   Adafruit_BusIO_RegisterBits data_rate =
//       Adafruit_BusIO_RegisterBits(&ctrl1, 3, 4);

//   data_rate.write((uint8_t)new_data_rate);
// }

// /******************* Adafruit_Sensor functions *****************/
// /*!
//  *     @brief  Updates the measurement data for all sensors simultaneously
//  */
// /**************************************************************************/
// void Adafruit_AS7341::Adafruit_AS7341::_read(void) {
//   // get raw readings

//   // TODO: Update for sensor types and enable multi-byte read if needed
//   // uint8_t pressure_addr = AS7341_PRESS_OUT_XL;
//   // uint8_t temp_addr = AS7341_TEMP_OUT_L;
//   // if (spi_dev) {
//   //   // for AS7341 SPI, addr[7] is r/w, addr[6] is auto increment
//   //   pressure_addr |= 0x40;
//   //   temp_addr |= 0x40;
//   // }

//   // TODO: use this block to manually pack bytes and fix sign
//   // Adafruit_BusIO_Register pressure_data = Adafruit_BusIO_Register(
//   //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, pressure_addr, 3);

//   // Adafruit_BusIO_Register temp_data = Adafruit_BusIO_Register(
//   //     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, temp_addr, 2);

//   // uint8_t buffer[3];

//   // temp_data.read(buffer, 2);
//   // int16_t raw_temp;

//   // raw_temp |= (int16_t)(buffer[1]);
//   // raw_temp <<= 8;
//   // raw_temp |= (int16_t)(buffer[0]);

//   // pressure_data.read(buffer, 3);
//   // int32_t raw_pressure;

//   // raw_pressure = (int32_t)buffer[2];
//   // raw_pressure <<= 8;
//   // raw_pressure |= (int32_t)(buffer[1]);
//   // raw_pressure <<= 8;
//   // raw_pressure |= (int32_t)(buffer[0]);

//   // // TODO: This can be done by casting to signed type
//   // if (raw_temp & 0x8000) {
//   //   raw_temp = raw_temp - 0xFFFF;
//   // }
//   // unscaled_temp = raw_temp;

//   // if (raw_pressure & 0x800000) {
//   //   raw_pressure = raw_pressure - 0xFFFFFF;
//   // }

//   Adafruit_BusIO_Register to_out =
//       Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
//                               (HTS221_T0_OUT | multi_byte_address_mask), 2);
//   Adafruit_BusIO_Register t1_out =
//       Adafruit_BusIO_Register(i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD,
//                               (HTS221_T1_OUT | multi_byte_address_mask), 2);

//   to_out.read(&T0_OUT);
//   t1_out.read(&T1_OUT);

//   unscaled_pressure = raw_pressure;
// }

// /*!
//     @brief  Gets an Adafruit Unified Sensor object for the presure sensor
//    component
//     @return Adafruit_Sensor pointer to pressure sensor
//  */
// Adafruit_Sensor *Adafruit_AS7341::getPressureSensor(void) {
//   return pressure_sensor;
// }

// /*!
//     @brief  Gets an Adafruit Unified Sensor object for the temp sensor
//     component
//     @return Adafruit_Sensor pointer to temperature sensor
//  */
// Adafruit_Sensor *Adafruit_AS7341::getTemperatureSensor(void) {
//   return temp_sensor;
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the pressure sensor and temperature values as sensor events
//     @param  pressure Sensor event object that will be populated with pressure
//    data
//     @param  temp Sensor event object that will be populated with temp data
//     @returns True
// */
// /**************************************************************************/
// bool Adafruit_AS7341::getEvent(sensors_event_t *pressure,
//                                sensors_event_t *temp) {
//   uint32_t t = millis();
//   _read();

//   // use helpers to fill in the events
//   fillPressureEvent(pressure, t);
//   fillTempEvent(temp, t);
//   return true;
// }

// void Adafruit_AS7341::Adafruit_AS7341::fillPressureEvent(sensors_event_t
// *pressure,
//                                         uint32_t timestamp) {
//   memset(pressure, 0, sizeof(sensors_event_t));
//   pressure->version = sizeof(sensors_event_t);
//   pressure->sensor_id = _sensorid_pressure;
//   pressure->type = SENSOR_TYPE_PRESSURE;
//   pressure->timestamp = timestamp;
//   pressure->pressure = (unscaled_pressure / 4096.0);
// }

// void Adafruit_AS7341::Adafruit_AS7341::fillTempEvent(sensors_event_t *temp,
// uint32_t timestamp) {
//   memset(temp, 0, sizeof(sensors_event_t));
//   temp->version = sizeof(sensors_event_t);
//   temp->sensor_id = _sensorid_temp;
//   temp->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
//   temp->timestamp = timestamp;
//   temp->temperature = (unscaled_temp / 480) + 42.5;
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the sensor_t data for the AS7341's tenperature
// */
// /**************************************************************************/
// void Adafruit_AS7341::Adafruit_AS7341_Pressure::getSensor(sensor_t *sensor) {
//   /* Clear the sensor_t object */
//   memset(sensor, 0, sizeof(sensor_t));

//   /* Insert the sensor name in the fixed length char array */
//   strncpy(sensor->name, "AS7341_P", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name) - 1] = 0;
//   sensor->version = 1;
//   sensor->sensor_id = _sensorID;
//   sensor->type = SENSOR_TYPE_PRESSURE;
//   sensor->min_delay = 0;
//   sensor->min_value = 260;
//   sensor->max_value = 1260;
//   // 4096 LSB = 1 hPa >>  1 LSB = 1/4096 hPa >> 1 LSB =  2.441e-4 hPa
//   sensor->resolution = 2.441e-4;
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the pressure as a standard sensor event
//     @param  event Sensor event object that will be populated
//     @returns True
// */
// /**************************************************************************/
// bool Adafruit_AS7341_Pressure::getEvent(sensors_event_t *event) {
//   _theAS7341->_read();
//   _theAS7341->fillPressureEvent(event, millis());

//   return true;
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the sensor_t data for the AS7341's tenperature
// */
// /**************************************************************************/
// void Adafruit_AS7341::Adafruit_AS7341_Temp::getSensor(sensor_t *sensor) {
//   /* Clear the sensor_t object */
//   memset(sensor, 0, sizeof(sensor_t));

//   /* Insert the sensor name in the fixed length char array */
//   strncpy(sensor->name, "AS7341_T", sizeof(sensor->name) - 1);
//   sensor->name[sizeof(sensor->name) - 1] = 0;
//   sensor->version = 1;
//   sensor->sensor_id = _sensorID;
//   sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
//   sensor->min_delay = 0;
//   sensor->min_value = -30;
//   sensor->max_value = 105;
//   // 480 LSB = 1°C >> 1 LSB = 1/480°C >> 1 LSB =  0.00208 °C
//   sensor->resolution = 0.00208;
// }

// /**************************************************************************/
// /*!
//     @brief  Gets the temperature as a standard sensor event
//     @param  event Sensor event object that will be populated
//     @returns True
// */
// /**************************************************************************/
// bool Adafruit_AS7341_Temp::getEvent(sensors_event_t *event) {
//   _theAS7341->_read();
//   _theAS7341->fillTempEvent(event, millis());

//   return true;
// }

// /**
//  * @brief Sets the polarity of the INT pin.
//  *
//  * @param active_low Set to true to make the pin active low
//  */
// void Adafruit_AS7341::Adafruit_AS7341::interruptsActiveLow(bool active_low) {
//   Adafruit_BusIO_Register ctrl3 = Adafruit_BusIO_Register(
//       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, AS7341_CTRL_REG3, 1);

//   Adafruit_BusIO_RegisterBits active_low_bit =
//       Adafruit_BusIO_RegisterBits(&ctrl3, 1, 7);
//   active_low_bit.write(active_low);
// }
