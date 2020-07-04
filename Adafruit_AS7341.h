/*!
 *  @file Adafruit_AS7341.h
 *
 * 	I2C Driver for the Adafruit AS7341 11-Channel Spectral Sensor
 *
 * 	This is a library for the Adafruit AS7341 breakout:
 * 	https://www.adafruit.com/products/45XX
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

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
#ifndef _ADAFRUIT_AS7341_H
#define _ADAFRUIT_AS7341_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI

#define AS7341_WHOAMI 0x92 ///< Chip ID register

#define AS7341_ASTATUS 0x60      ///<
#define AS7341_CH0_DATA_L 0x61   ///<
#define AS7341_CH0_DATA_H 0x62   ///<
#define AS7341_ITIME_L 0x63      ///<
#define AS7341_ITIME_M 0x64      ///<
#define AS7341_ITIME_H 0x65      ///<
#define AS7341_CH1_DATA_L 0x66   ///<
#define AS7341_CH1_DATA_H 0x67   ///<
#define AS7341_CH2_DATA_L 0x68   ///<
#define AS7341_CH2_DATA_H 0x69   ///<
#define AS7341_CH3_DATA_L 0x6A   ///<
#define AS7341_CH3_DATA_H 0x6B   ///<
#define AS7341_CH4_DATA_L 0x6C   ///<
#define AS7341_CH4_DATA_H 0x6D   ///<
#define AS7341_CH5_DATA_L 0x6E   ///<
#define AS7341_CH5_DATA_H 0x6F   ///<
#define AS7341_CONFIG 0x70       ///<
#define AS7341_STAT 0x71         ///<
#define AS7341_EDGE 0x72         ///<
#define AS7341_GPIO 0x73         ///<
#define AS7341_LED 0x74          ///<
#define AS7341_ENABLE 0x80       ///<
#define AS7341_ATIME 0x81        ///<
#define AS7341_WTIME 0x83        ///<
#define AS7341_SP_LOW_TH_L 0x84  ///<
#define AS7341_SP_LOW_TH_H 0x85  ///<
#define AS7341_SP_HIGH_TH_L 0x86 ///<
#define AS7341_SP_HIGH_TH_H 0x87 ///<
#define AS7341_AUXID 0x90        ///<
#define AS7341_REVID 0x91        ///<
#define AS7341_ID 0x92           ///<
#define AS7341_STATUS 0x93       ///<
#define AS7341_ASTATUS 0x94      ///<
#define AS7341_CH0_DATA_L 0x95   ///<
#define AS7341_CH0_DATA_H 0x96   ///<
#define AS7341_CH1_DATA_L 0x97   ///<
#define AS7341_CH1_DATA_H 0x98   ///<
#define AS7341_CH2_DATA_L 0x99   ///<
#define AS7341_CH2_DATA_H 0x9A   ///<
#define AS7341_CH3_DATA_L 0x9B   ///<
#define AS7341_CH3_DATA_H 0x9C   ///<
#define AS7341_CH4_DATA_L 0x9D   ///<
#define AS7341_CH4_DATA_H 0x9E   ///<
#define AS7341_CH5_DATA_L 0x9F   ///<
#define AS7341_CH5_DATA_H 0xA0   ///<
#define AS7341_STATUS2 0xA3      ///<
#define AS7341_STATUS3 0xA4      ///<
#define AS7341_STATUS5 0xA6      ///<
#define AS7341_STATUS6 0xA7      ///<
#define AS7341_CFG0 0xA9         ///<
#define AS7341_CFG1 0xAA         ///<
#define AS7341_CFG3 0xAC         ///<
#define AS7341_CFG6 0xAF         ///<
#define AS7341_CFG8 0xB1         ///<
#define AS7341_CFG9 0xB2         ///<
#define AS7341_CFG10 0xB3        ///<
#define AS7341_CFG12 0xB5        ///<
#define AS7341_PERS 0xBD         ///<
#define AS7341_GPIO 2 0xBE       ///<
#define AS7341_ASTEP_L 0xCA      ///<
#define AS7341_ASTEP_H 0xCB      ///<
#define AS7341_AGC_GAIN_MAX 0xCF ///<
#define AS7341_AZ_CONFIG 0xD6    ///<
#define AS7341_FD_TIME1 0xD8     ///<
#define AS7341_FD_TIME2 0xDA     ///<
#define AS7341_FD_CFG0 0xD7      ///<
#define AS7341_FD_STATUS 0xDB    ///<
#define AS7341_INTENAB 0xF9      ///<
#define AS7341_CONTROL 0xFA      ///<
#define AS7341_FIFO_MAP 0xFC     ///<
#define AS7341_FIFO_LVL 0xFD     ///<
#define AS7341_FDATA_L 0xFE      ///<
#define AS7341_FDATA_H 0xFF      ///<

///////////////////////////////////////////////////////////////
/**
 * @brief
 *
 * Allowed values for `setDataRate`.
 */
// typedef enum {
//   AS7341_RATE_ONE_SHOT,
//   AS7341_RATE_1_HZ,
//   AS7341_RATE_7_HZ,
//   AS7341_RATE_12_5_HZ,
//   AS7341_RATE_25_HZ,
// } as7341_rate_t;

class Adafruit_AS7341;

// /** Adafruit Unified Sensor interface for temperature component of AS7341 */
// class Adafruit_AS7341_Temp : public Adafruit_Sensor {
// public:
//   /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
//       @param parent A pointer to the AS7341 class */
//   Adafruit_AS7341_Temp(Adafruit_AS7341 *parent) { _theAS7341 = parent; }
//   bool getEvent(sensors_event_t *);
//   void getSensor(sensor_t *);

// private:
//   int _sensorID = as7341;
//   Adafruit_AS7341 *_theAS7341 = NULL;
// };

// /** Adafruit Unified Sensor interface for the pressure sensor component of
//  * AS7341
//  */
// class Adafruit_AS7341_Pressure : public Adafruit_Sensor {
// public:
//   /** @brief Create an Adafruit_Sensor compatible object for the pressure
//   sensor
//       @param parent A pointer to the AS7341 class */
//   Adafruit_AS7341_Pressure(Adafruit_AS7341 *parent) { _theAS7341 = parent; }
//   bool getEvent(sensors_event_t *);
//   void getSensor(sensor_t *);

// private:
//   int _sensorID = as7341 + 1;
//   Adafruit_AS7341 *_theAS7341 = NULL;
// };

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the AS7341 11-Channel Spectral Sensor
 */
class Adafruit_AS7341 {
public:
  Adafruit_AS7341();
  ~Adafruit_AS7341();

  bool begin(uint8_t i2c_addr = AS7341_I2CADDR_DEFAULT, TwoWire *wire = &Wire,
             int32_t sensor_id = 0);

  void setASTEP(uint16_t astep_value);
  void setATIME(byte value);
  void setGAIN(byte value);

  void readRawValuesMode1(void);
  void readRawValuesMode2(void);

  void flickerDetection(void); // merge with 1k
  void flickerDetection1K(void);

  void FDConfig(void);
  bool getFdMeasReady();

  int8_t getFlickerValue(void);

  void F1F4_Clear_NIR(void);
  void F5F8_Clear_NIR(void);

  void powerEnable(bool enable_power);
  bool enableSpectralMeasurement(bool enable_measurement);

  void writeRegister(byte addr, byte val);
  byte readRegister(byte addr);
  uint16_t readTwoRegister1(byte addr);
  void readRegisterPrint(byte addr);

  void enableSMUX(void);
  void SmuxConfigRAM(void);

  bool getIsDataReady();

  bool getEvent(sensors_event_t *pressure, sensors_event_t *temp);
  // void interruptsActiveLow(bool active_low);
  // as7341_rate_t getDataRate(void);
  // void setDataRate(as7341_rate_t data_rate);

  // Adafruit_Sensor *getTemperatureSensor(void);
  // Adafruit_Sensor *getPressureSensor(void);

protected:
  void _read(void);
  virtual bool _init(int32_t sensor_id);

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  // float unscaled_temp,   ///< Last reading's temperature (C) before scaling
  // unscaled_pressure; ///< Last reading's pressure (hPa) before scaling

  // uint16_t _sensorid_pressure, ///< ID number for pressure
  // _sensorid_temp;          ///< ID number for temperature

  // Adafruit_AS7341_Temp *temp_sensor = NULL; ///< Temp sensor data object
  // Adafruit_AS7341_Pressure *pressure_sensor =
  // NULL; ///< Pressure sensor data object

private:
  // friend class Adafruit_AS7341_Temp;     ///< Gives access to private members
  // to
  //                                        ///< Temp data object
  // friend class Adafruit_AS7341_Pressure; ///< Gives access to private
  //                                        ///< members to Pressure data
  //                                        ///< object

  // void fillPressureEvent(sensors_event_t *pressure, uint32_t timestamp);
  // void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
};

#endif
