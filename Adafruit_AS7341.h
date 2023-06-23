/*!
 *  @file Adafruit_AS7341.h

 *  @mainpage Adafruit AS7341 11-Channel Spectral Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Library for the AS7341 11-Channel Spectral Sensor
 *
 * 	This is a library for the Adafruit AS7341 breakout:
 * 	https://www.adafruit.com/product/4698
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
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

/*
  This library is adapted from an example with the following Copyright and
  Warranty:

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
#include <Wire.h>
#define AS7341_I2CADDR_DEFAULT 0x39 ///< AS7341 default i2c address
#define AS7341_CHIP_ID 0x09         ///< AS7341 default device id from WHOAMI

#define AS7341_WHOAMI 0x92 ///< Chip ID register

#define AS7341_ASTATUS 0x60     ///< AS7341_ASTATUS (unused)
#define AS7341_CH0_DATA_L_ 0x61 ///< AS7341_CH0_DATA_L (unused)
#define AS7341_CH0_DATA_H_ 0x62 ///< AS7341_CH0_DATA_H (unused)
#define AS7341_ITIME_L 0x63     ///< AS7341_ITIME_L (unused)
#define AS7341_ITIME_M 0x64     ///< AS7341_ITIME_M (unused)
#define AS7341_ITIME_H 0x65     ///< AS7341_ITIME_H (unused)
#define AS7341_CONFIG 0x70 ///< Enables LED control and sets light sensing mode
#define AS7341_STAT 0x71   ///< AS7341_STAT (unused)
#define AS7341_EDGE 0x72   ///< AS7341_EDGE (unused)
#define AS7341_GPIO 0x73   ///< Connects photo diode to GPIO or INT pins
#define AS7341_LED 0x74    ///< LED Register; Enables and sets current limit
#define AS7341_ENABLE                                                          \
  0x80 ///< Main enable register. Controls SMUX, Flicker Detection, Spectral
       ///< Measurements and Power
#define AS7341_ATIME 0x81       ///< Sets ADC integration step count
#define AS7341_WTIME 0x83       ///< AS7341_WTIME (unused)
#define AS7341_SP_LOW_TH_L 0x84 ///< Spectral measurement Low Threshold low byte
#define AS7341_SP_LOW_TH_H                                                     \
  0x85 ///< Spectral measurement Low Threshold high byte
#define AS7341_SP_HIGH_TH_L                                                    \
  0x86 ///< Spectral measurement High Threshold low byte
#define AS7341_SP_HIGH_TH_H                                                    \
  0x87                    ///< Spectral measurement High Threshold low byte
#define AS7341_AUXID 0x90 ///< AS7341_AUXID (unused)
#define AS7341_REVID 0x91 ///< AS7341_REVID (unused)
#define AS7341_ID 0x92    ///< AS7341_ID (unused)
#define AS7341_STATUS                                                          \
  0x93 ///< Interrupt status registers. Indicates the occourance of an interrupt
#define AS7341_ASTATUS_ 0x94   ///< AS7341_ASTATUS, same as 0x60 (unused)
#define AS7341_CH0_DATA_L 0x95 ///< ADC Channel Data
#define AS7341_CH0_DATA_H 0x96 ///< ADC Channel Data
#define AS7341_CH1_DATA_L 0x97 ///< ADC Channel Data
#define AS7341_CH1_DATA_H 0x98 ///< ADC Channel Data
#define AS7341_CH2_DATA_L 0x99 ///< ADC Channel Data
#define AS7341_CH2_DATA_H 0x9A ///< ADC Channel Data
#define AS7341_CH3_DATA_L 0x9B ///< ADC Channel Data
#define AS7341_CH3_DATA_H 0x9C ///< ADC Channel Data
#define AS7341_CH4_DATA_L 0x9D ///< ADC Channel Data
#define AS7341_CH4_DATA_H 0x9E ///< ADC Channel Data
#define AS7341_CH5_DATA_L 0x9F ///< ADC Channel Data
#define AS7341_CH5_DATA_H 0xA0 ///< ADC Channel Data
#define AS7341_STATUS2 0xA3 ///< Measurement status flags; saturation, validity
#define AS7341_STATUS3                                                         \
  0xA4 ///< Spectral interrupt source, high or low threshold
#define AS7341_STATUS5 0xA6 ///< AS7341_STATUS5 (unused)
#define AS7341_STATUS6 0xA7 ///< AS7341_STATUS6 (unused)
#define AS7341_CFG0                                                            \
  0xA9 ///< Sets Low power mode, Register bank, and Trigger lengthening
#define AS7341_CFG1 0xAA ///< Controls ADC Gain
#define AS7341_CFG3 0xAC ///< AS7341_CFG3 (unused)
#define AS7341_CFG6 0xAF ///< Used to configure Smux
#define AS7341_CFG8 0xB1 ///< AS7341_CFG8 (unused)
#define AS7341_CFG9                                                            \
  0xB2 ///< Enables flicker detection and smux command completion system
       ///< interrupts
#define AS7341_CFG10 0xB3 ///< AS7341_CFG10 (unused)
#define AS7341_CFG12                                                           \
  0xB5 ///< Spectral threshold channel for interrupts, persistence and auto-gain
#define AS7341_PERS                                                            \
  0xBD ///< Number of measurement cycles outside thresholds to trigger an
       ///< interupt
#define AS7341_GPIO2                                                           \
  0xBE ///< GPIO Settings and status: polarity, direction, sets output, reads
       ///< input
#define AS7341_ASTEP_L 0xCA      ///< Integration step size ow byte
#define AS7341_ASTEP_H 0xCB      ///< Integration step size high byte
#define AS7341_AGC_GAIN_MAX 0xCF ///< AS7341_AGC_GAIN_MAX (unused)
#define AS7341_AZ_CONFIG 0xD6    ///< AS7341_AZ_CONFIG (unused)
#define AS7341_FD_TIME1 0xD8 ///< Flicker detection integration time low byte
#define AS7341_FD_TIME2 0xDA ///< Flicker detection gain and high nibble
#define AS7341_FD_CFG0 0xD7  ///< AS7341_FD_CFG0 (unused)
#define AS7341_FD_STATUS                                                       \
  0xDB ///< Flicker detection status; measurement valid, saturation, flicker
       ///< type
#define AS7341_INTENAB 0xF9  ///< Enables individual interrupt types
#define AS7341_CONTROL 0xFA  ///< Auto-zero, fifo clear, clear SAI active
#define AS7341_FIFO_MAP 0xFC ///< AS7341_FIFO_MAP (unused)
#define AS7341_FIFO_LVL 0xFD ///< AS7341_FIFO_LVL (unused)
#define AS7341_FDATA_L 0xFE  ///< AS7341_FDATA_L (unused)
#define AS7341_FDATA_H 0xFF  ///< AS7341_FDATA_H (unused)

#define AS7341_SPECTRAL_INT_HIGH_MSK                                           \
  0b00100000 ///< bitmask to check for a high threshold interrupt
#define AS7341_SPECTRAL_INT_LOW_MSK                                            \
  0b00010000 ///< bitmask to check for a low threshold interrupt

/**
 * @brief Allowable gain multipliers for `setGain`
 *
 */
typedef enum {
  AS7341_GAIN_0_5X,
  AS7341_GAIN_1X,
  AS7341_GAIN_2X,
  AS7341_GAIN_4X,
  AS7341_GAIN_8X,
  AS7341_GAIN_16X,
  AS7341_GAIN_32X,
  AS7341_GAIN_64X,
  AS7341_GAIN_128X,
  AS7341_GAIN_256X,
  AS7341_GAIN_512X,
} as7341_gain_t;

/**
 * @brief Available SMUX configuration commands
 *
 */
typedef enum {
  AS7341_SMUX_CMD_ROM_RESET, ///< ROM code initialization of SMUX
  AS7341_SMUX_CMD_READ,      ///< Read SMUX configuration to RAM from SMUX chain
  AS7341_SMUX_CMD_WRITE, ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_cmd_t;
/**
 * @brief ADC Channel specifiers for configuration
 *
 */
typedef enum {
  AS7341_ADC_CHANNEL_0,
  AS7341_ADC_CHANNEL_1,
  AS7341_ADC_CHANNEL_2,
  AS7341_ADC_CHANNEL_3,
  AS7341_ADC_CHANNEL_4,
  AS7341_ADC_CHANNEL_5,
} as7341_adc_channel_t;
/**
 * @brief Spectral Channel specifiers for configuration and reading
 *
 */
typedef enum {
  AS7341_CHANNEL_415nm_F1,
  AS7341_CHANNEL_445nm_F2,
  AS7341_CHANNEL_480nm_F3,
  AS7341_CHANNEL_515nm_F4,
  AS7341_CHANNEL_CLEAR_0,
  AS7341_CHANNEL_NIR_0,
  AS7341_CHANNEL_555nm_F5,
  AS7341_CHANNEL_590nm_F6,
  AS7341_CHANNEL_630nm_F7,
  AS7341_CHANNEL_680nm_F8,
  AS7341_CHANNEL_CLEAR,
  AS7341_CHANNEL_NIR,
} as7341_color_channel_t;

/**
 * @brief The number of measurement cycles with spectral data outside of a
 * threshold required to trigger an interrupt
 *
 */
typedef enum {
  AS7341_INT_COUNT_ALL, ///< 0
  AS7341_INT_COUNT_1,   ///< 1
  AS7341_INT_COUNT_2,   ///< 2
  AS7341_INT_COUNT_3,   ///< 3
  AS7341_INT_COUNT_5,   ///< 4
  AS7341_INT_COUNT_10,  ///< 5
  AS7341_INT_COUNT_15,  ///< 6
  AS7341_INT_COUNT_20,  ///< 7
  AS7341_INT_COUNT_25,  ///< 8
  AS7341_INT_COUNT_30,  ///< 9
  AS7341_INT_COUNT_35,  ///< 10
  AS7341_INT_COUNT_40,  ///< 11
  AS7341_INT_COUNT_45,  ///< 12
  AS7341_INT_COUNT_50,  ///< 13
  AS7341_INT_COUNT_55,  ///< 14
  AS7341_INT_COUNT_60,  ///< 15
} as7341_int_cycle_count_t;

/**
 * @brief Pin directions to set how the GPIO pin is to be used
 *
 */
typedef enum {
  AS7341_GPIO_OUTPUT, ///< THhe GPIO pin is configured as an open drain output
  AS7341_GPIO_INPUT,  ///< The GPIO Pin is set as a high-impedence input
} as7341_gpio_dir_t;

/**
 * @brief Wait states for async reading
 */
typedef enum {
  AS7341_WAITING_START, //
  AS7341_WAITING_LOW,   //
  AS7341_WAITING_HIGH,  //
  AS7341_WAITING_DONE,  //
} as7341_waiting_t;

class Adafruit_AS7341;

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

  bool setASTEP(uint16_t astep_value);
  bool setATIME(uint8_t atime_value);
  bool setGain(as7341_gain_t gain_value);

  uint16_t getASTEP();
  uint8_t getATIME();
  as7341_gain_t getGain();

  long getTINT();
  float toBasicCounts(uint16_t raw);

  bool readAllChannels(void);
  bool readAllChannels(uint16_t *readings_buffer);
  void delayForData(int waitTime = 0);
  uint16_t readChannel(as7341_adc_channel_t channel);
  uint16_t getChannel(as7341_color_channel_t channel);

  bool startReading(void);
  bool checkReadingProgress();
  bool getAllChannels(uint16_t *readings_buffer);

  uint16_t detectFlickerHz(void);

  void setup_F1F4_Clear_NIR(void);
  void setup_F5F8_Clear_NIR(void);

  void powerEnable(bool enable_power);
  bool enableSpectralMeasurement(bool enable_measurement);

  bool setHighThreshold(uint16_t high_threshold);
  bool setLowThreshold(uint16_t low_threshold);

  uint16_t getHighThreshold(void);
  uint16_t getLowThreshold(void);

  bool enableSpectralInterrupt(bool enable_int);
  bool enableSystemInterrupt(bool enable_int);

  bool setAPERS(as7341_int_cycle_count_t cycle_count);
  bool setSpectralThresholdChannel(as7341_adc_channel_t channel);

  uint8_t getInterruptStatus(void);
  bool clearInterruptStatus(void);

  bool spectralInterruptTriggered(void);
  uint8_t spectralInterruptSource(void);
  bool spectralLowTriggered(void);
  bool spectralHighTriggered(void);

  bool enableLED(bool enable_led);
  bool setLEDCurrent(uint16_t led_current_ma);
  uint16_t getLEDCurrent(void);

  void disableAll(void);

  bool getIsDataReady();
  bool setBank(bool low); // low true gives access to 0x60 to 0x74

  as7341_gpio_dir_t getGPIODirection(void);
  bool setGPIODirection(as7341_gpio_dir_t gpio_direction);
  bool getGPIOInverted(void);
  bool setGPIOInverted(bool gpio_inverted);
  bool getGPIOValue(void);
  bool setGPIOValue(bool);

protected:
  virtual bool _init(int32_t sensor_id);
  uint8_t last_spectral_int_source =
      0; ///< The value of the last reading of the spectral interrupt source
         ///< register

  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

private:
  bool enableSMUX(void);
  bool enableFlickerDetection(bool enable_fd);
  void FDConfig(void);
  int8_t getFlickerDetectStatus(void);
  bool setSMUXCommand(as7341_smux_cmd_t command);
  void writeRegister(byte addr, byte val);
  void setSMUXLowChannels(bool f1_f4);
  uint16_t _channel_readings[12];
  as7341_waiting_t _readingState;
};

#endif
