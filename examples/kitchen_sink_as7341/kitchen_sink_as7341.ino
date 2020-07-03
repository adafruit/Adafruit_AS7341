
/*

  This is a Hello world example code written for the AS7241 XWing Spectral
  Sensor I2C interface with Arduino µC. The main idea is to get fimilar with the
  register configuration. This code helps to learn basic settings and procedure
  to read out raw values with different SMUX configuration. Also defined the
  procedure to set the default flicker detection for 100 and 120 Hz.

  Written by Sijo John @ ams AG, Application Support in October, 2018

  Development environment specifics: Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/
#include <Wire.h>

// I2C device address - 0x39
#define _i2cAddr (0x39)

void setup() {

  // Initiate the Wire library and join the I2C bus as a master or slave
  Wire.begin();

  // communication with the host computer serial monitor
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
}

void loop() {

 // Sets the Atime for integration time from 0 to 255 in register (0x81),
 // integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS
 setATIME(byte(0x64));

 // Sets the Astep for integration time from 0 to 65535 in register (0xCA[7:0])
 // and (0xCB[15:8]), integration time = (ATIME + 1) * (ASTEP + 1) * 2.78µS
 setASTEP(byte(0xE7), byte(0x03));

 // Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
 setGAIN(byte(0x09));

 // Function defined to read out channels with SMUX configration 1- F1-F4,
 // Clear, NIR
 ReadRawValuesMode1();

 // Function defined to read out channels with SMUX configration 2- F5-F8,
 // Clear, NIR
 ReadRawValuesMode2();
 delay(1000);
//
//  // Function detects Flicker for 100 and 120 Hz
//  flickerDetection();

  flickerDetection1K();

  // reading the flicker status in FD_STATUS register 0xDB
  // data=0x2c=0b00101100  FD_STATUS(fd_measurement_valid=1
  // fd_120Hz_flicker_valid=1 fd_100Hz_flicker_valid=1) data=0x2d=0b00101101
  // FD_STATUS(fd_measurement_valid=1 fd_1200Hz_flicker_valid=1
  // fd_1000Hz_flicker_valid=1 fd_1000Hz_flicker) data=0x2e=0b00101110
  // FD_STATUS(fd_measurement_valid=1 fd_1200Hz_flicker_valid=1
  // fd_1000Hz_flicker_valid=1 fd_1200Hz_flicker)

  int flicker_value = readRegister(byte(0xDB));
  //             Serial.print("Flicker Status-");
  //             Serial.println(flicker_value);

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

// <summary>
// Read two consecutive i2c registers
// <summary>
// param name = "addr">First register address of two consecutive registers to be
// read param name = "_i2cAddr">Device address 0x39

uint16_t readTwoRegister1(byte addr) {
  uint8_t readingL;
  uint16_t readingH;
  uint16_t reading = 0;
  Wire.beginTransmission(_i2cAddr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(_i2cAddr, 2);

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
// "_i2cAddr">Device address 0x39

void writeRegister(byte addr, byte val) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(addr);
  Wire.write(val);
  Wire.endTransmission();
}

/*----- Register configuration  -----*/

// <summary>
// Setting the PON (Power on) bit on the chip (bit0 at register ENABLE 0x80)
// Attention: This function clears only the PON bit in ENABLE register and keeps
// the other bits <summary>

void PON() {

  byte regVal = readRegister(byte(0x80));
  byte temp = regVal;
  regVal = regVal & 0xFE;
  regVal = regVal | 0x01;
  writeRegister(byte(0x80), byte(regVal));
}

// <summary>
// Write SMUX configration from RAM to set SMUX chain in CFG6 register 0xAF
// <summary>

void SmuxConfigRAM() { writeRegister(byte(0xAF), byte(0x10)); }

// <summary>
// Setting the SP_EN (spectral measurement enabled) bit on the chip (bit 1 in
// register ENABLE) <summary> <param name="isEnable">Enabling (true) or
// disabling (false) the SP_EN bit</param>

void SpEn(bool isEnable) {

  byte regVal = readRegister(byte(0x80));
  byte temp = regVal;
  regVal = regVal & 0xFD;

  if (isEnable == true) {
    regVal = regVal | 0x02;
  } else {
    regVal = temp & 0xFD;
  }

  writeRegister(byte(0x80), byte(regVal));
}

// <summary>
// Starting the SMUX command via enabling the SMUXEN bit (bit 4) in register
// ENABLE 0x80 The SMUXEN bit gets cleared automatically as soon as SMUX
// operation is finished <summary>

void SMUXEN() {

  byte regVal = readRegister(byte(0x80));
  byte temp = regVal;
  regVal = regVal & 0xEF;
  regVal = regVal | 0x10;
  writeRegister(byte(0x80), byte(regVal));
}

// <summary>
// Reading and Polling the the SMUX Enable bit in Enable Register 0x80
// The SMUXEN bit gets cleared automatically as soon as SMUX operation is
// finished <summary>

bool getSmuxEnabled() {

  bool isEnabled = false;
  byte regVal = readRegister(byte(0x80));

  if ((regVal & 0x10) == 0x10) {
    return isEnabled = true;
  }

  else {
    return isEnabled = false;
  }
}

// <summary>
// Reading and Polling the the AVALID bit in Status 2 Register 0xA3,if the
// spectral measurement is ready or busy. True indicates that a cycle is
// completed since the last readout of the Raw Data register <summary>

bool getIsDataReady() {
  bool isDataReady = false;
  byte regVal = readRegister(byte(0xA3));

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

bool getFdMeasReady() {
  bool isFdmeasReady = false;
  byte regVal = readRegister(byte(0xDB));

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

void F1F4_Clear_NIR() {
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

void F5F8_Clear_NIR() {
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

void FDConfig() {
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

void setATIME(byte value) { writeRegister(byte(0x81), value); }

//<summary>
// Sets the ASTEP for integration time from 0 to 65535, integration time =
// (ATIME + 1) * (ASTEP + 1) * 2.78µS
//<summary>
// param name = "value1,"> Defines the lower byte[7:0] of the base step time
// written to ASTEP register 0xCA param name = "value2,"> Defines the higher
// byte[15:8] of the base step time written to ASTEP register 0xCB

void setASTEP(byte value1, byte value2) {

  // astep[7:0]
  writeRegister(byte(0xCA), value1);

  // astep[15:8]
  writeRegister(byte(0xCB), value2);
}

//<summary>
// Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
//<summary>
// param name = "value"> integer value from 0 to 10 written to AGAIN register
// 0xAA

void setGAIN(byte value) { writeRegister(byte(0xAA), value); }

/*----- Device ID, revision ID and auxiliary ID are read(These function are not
 * implemented in main code. This is to give just an idea regarding these
 * register) -----*/

//<summary>
// Reads Identification number register in 0x92
//<summary>

void readID() { readRegister(byte(0x92)); }

//<summary>
// Reads Revision identification number in 0x91
//<summary>

void readREVID() { readRegister(byte(0x91)); }

//<summary>
// Reads Auxiliary identification number in 0x93
//<summary>
void readAUXID() { readRegister(byte(0x93)); }

/*----- Function to detect Flickering at 100 and 120 Hz(default detection in
 * XWing Sensor) -----*/

//<summary>
// Executing a flicker measurement cycle, displaying the status from FD_Status
// register
//<summary>

void flickerDetection() {
  bool isEnabled = true;
  bool isFdmeasReady = false;

  writeRegister(byte(0x80), byte(0x00));

  // Setting the PON bit in Enable register 0x80

  PON();

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)

  SmuxConfigRAM();

  // Write new configuration to all the 20 registers for detecting Flicker

  FDConfig();

  // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE

  SMUXEN();

  // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN
  // bit -> if it is 0 SMUX command is started

  while (isEnabled) {
    isEnabled = getSmuxEnabled();
  }

  // Enable SP_EN bit
  SpEn(true);

  /*----- Functions for setting Flicker Sample, Flicker time, Flicker Gain (not
   * implemented for default flicker detection)------*/
  //            writeRegister(byte(0xD7), byte(0x21)); //33 default value,
  //            function for setting for Fd_sample and Fd_compare_value
  //
  //            writeRegister(byte(0xD8), byte(0x68)); //104 default value,
  //            function for setting for Fd_time lower bit(7:0)
  //
  //            writeRegister(byte(0xDA), byte(0x49)); //73 default value,
  //            function for setting for fd_gain and fd_time higher bit(10:8)

  // Function to set the Flicker detection via enabling the fden bit in 0x80
  // register

  writeRegister(byte(0x80), byte(0x41));
  delay(500);

  // reading the flicker status in FD_STATUS register 0xDB

  int flicker_value = readRegister(byte(0xDB));
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

  writeRegister(byte(0x80), byte(0x01));

  Serial.println("");
}

/*##################### 1K Flicker detect
 * #######################################*/

// <summary>
// To detect the target frequency of 1kHz and further 1.2kHz Flickering
// <summary>
void flickerDetection1K() {

  // RAM_BANK 0 select which RAM bank to access in register addresses 0x00-0x7f
  writeRegister(byte(0xA9), byte(0x00));

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
  writeRegister(byte(0xA9), byte(0x01));

  // write new coefficients to detect the 1000Hz and 1200Hz - part 1
  writeRegister(byte(0x06), byte(0x54));
  writeRegister(byte(0x07), byte(0x1A));
  writeRegister(byte(0x10), byte(0xB3));
  writeRegister(byte(0x11), byte(0x35));
  writeRegister(byte(0x1A), byte(0x2F));
  writeRegister(byte(0x1B), byte(0x1A));

  writeRegister(byte(0xA9), byte(0x01));

  // select RAM coefficients for flicker detection by setting
  // fd_disable_constant_init to „1“ (FD_CFG0 register) in FD_CFG0 register -
  // 0xd7  fd_disable_constant_init=1 fd_samples=4
  writeRegister(byte(0xD7), byte(0x60));
  // readRegisterPrint(byte(0xD7));

  // in FD_CFG1 register - 0xd8 fd_time(7:0) = 0x40
  writeRegister(byte(0xD8), byte(0x40));
  // readRegisterPrint(byte(0xD8));

  // in FD_CFG2 register - 0xd9  fd_dcr_filter_size=1 fd_nr_data_sets(2:0)=5
  writeRegister(byte(0xD9), byte(0x25));
  // readRegisterPrint(byte(0xD9));

  // in FD_CFG3 register - 0xda fd_gain=9
  writeRegister(byte(0xDA), byte(0x48));
  // readRegisterPrint(byte(0xDA));

  // in CFG9 register - 0xb2 sien_fd=1
  writeRegister(byte(0xB2), byte(0x40));
  // readRegisterPrint(byte(0xB2));

  // in ENABLE - 0x80  fden=1 and pon=1 are enabled
  writeRegister(byte(0x80), byte(0x41));
  // readRegisterPrint(byte(0x80));
}

/*#####################  END 1K Flicker detect
 * #######################################*/

/*----- Function defined to read out channels with SMUX configration 1 -----*/

//<summary>
// Executing raw data measurement cycle for 6 channels F1,F2,F3,F4,NIR,Clear
//<summary>

void ReadRawValuesMode1() {

  bool isEnabled = true;
  bool isDataReady = false;

  // Setting the PON bit in Enable register 0x80

  PON();

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)

  SmuxConfigRAM();

  // Write new configuration to all the 20 registers

  F1F4_Clear_NIR();

  // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE

  SMUXEN();

  // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN
  // bit -> if it is 0 SMUX command is started

  while (isEnabled) {
    isEnabled = getSmuxEnabled();
  }

  // Enable SP_EN bit

  SpEn(true);

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

void ReadRawValuesMode2() {
  bool isEnabled = true;
  bool isDataReady = false;

  // Setting the PON bit in Enable register 0x80

  PON();

  // Disable SP_EN bit

  SpEn(false);

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)

  SmuxConfigRAM();

  // Write new configuration to all the 20 registers for reading channels from
  // F5-F8, Clear and NIR

  F5F8_Clear_NIR();

  // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE

  SMUXEN();

  // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN
  // bit -> if it is 0 SMUX command is started

  while (isEnabled)

  {
    isEnabled = getSmuxEnabled();
  }

  // Enable SP_EN bit

  SpEn(true);

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

void readRegisterPrint(byte addr) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(_i2cAddr, 1);

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
// param name = "_i2cAddr">Device address 0x39

byte readRegister(byte addr) {
  Wire.beginTransmission(_i2cAddr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(_i2cAddr, 1);

  if (Wire.available()) {
    // Serial.println(Wire.read());
    return (Wire.read());
  }

  else {
    Serial.println("I2C Error");
    return (0xFF); // Error
  }
}