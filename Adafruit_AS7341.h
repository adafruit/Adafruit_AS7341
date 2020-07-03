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
// "requires_busio": "y",
//   "requires_sensor": "y",
#ifndef _ADAFRUIT_AS7341_H
#define _ADAFRUIT_AS7341_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09 ///< AS7341 default device id from WHOAMI

#define AS7341_WHOAMI 0x92 ///< Chip ID register

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
//   /** @brief Create an Adafruit_Sensor compatible object for the pressure sensor
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

  bool begin_I2C(uint8_t i2c_addr = AS7341_I2CADDR_DEFAULT,
                 TwoWire *wire = &Wire, int32_t sensor_id = 0);

  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 int32_t sensor_id = 0);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, int32_t sensor_id = 0);

  void reset(void);
  // void interruptsActiveLow(bool active_low);

  as7341_rate_t getDataRate(void);

  // void setDataRate(as7341_rate_t data_rate);
  bool getEvent(sensors_event_t *pressure, sensors_event_t *temp);

  // Adafruit_Sensor *getTemperatureSensor(void);
  // Adafruit_Sensor *getPressureSensor(void);

protected:
  void _read(void);
  virtual bool _init(int32_t sensor_id);

  // float unscaled_temp,   ///< Last reading's temperature (C) before scaling
      // unscaled_pressure; ///< Last reading's pressure (hPa) before scaling

  // uint16_t _sensorid_pressure, ///< ID number for pressure
      // _sensorid_temp;          ///< ID number for temperature

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  // Adafruit_AS7341_Temp *temp_sensor = NULL; ///< Temp sensor data object
  // Adafruit_AS7341_Pressure *pressure_sensor =
      // NULL; ///< Pressure sensor data object

private:
  // friend class Adafruit_AS7341_Temp;     ///< Gives access to private members to
  //                                        ///< Temp data object
  // friend class Adafruit_AS7341_Pressure; ///< Gives access to private
  //                                        ///< members to Pressure data
  //                                        ///< object

  // void fillPressureEvent(sensors_event_t *pressure, uint32_t timestamp);
  // void fillTempEvent(sensors_event_t *temp, uint32_t timestamp);
};

#endif