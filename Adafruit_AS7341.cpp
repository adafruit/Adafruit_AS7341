/*!
 *  @file Adafruit_AS7341.cpp
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
 *  Copyright 2020 Bryan Siepert for Adafruit Industries
 *
 * 	BSD (see license.txt)
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
/**
 * @brief Returns the flicker detection status
 *
 * @return int8_t
 */
int8_t Adafruit_AS7341::getFlickerDetectStatus(void) {
  Adafruit_BusIO_Register flicker_val =
      Adafruit_BusIO_Register(i2c_dev, AS7341_FD_STATUS);
  return (int8_t)flicker_val.read();
}

/**
 * @brief Returns the ADC data for a given channel
 *
 * @param channel The ADC channel to read
 * @return uint16_t The measured data for the currently configured sensor
 */
uint16_t Adafruit_AS7341::readChannel(as7341_adc_channel_t channel) {
  // each channel has two bytes, so offset by two for each next channel
  Adafruit_BusIO_Register channel_data_reg = Adafruit_BusIO_Register(
      i2c_dev, (AS7341_CH0_DATA_L + 2 * channel), 2, LSBFIRST);

  return channel_data_reg.read();
}

/**
 * @brief Returns the reading data for the specified color channel
 *
 *  call `readAllChannels` before reading to update the stored readings
 *
 * @param channel The color sensor channel to read
 * @return uint16_t The measured data for the selected sensor channel
 */
uint16_t Adafruit_AS7341::getChannel(as7341_color_channel_t channel) {
  return _channel_readings[channel];
}

/**
 * @brief fills the provided buffer with the current measurements for Spectral
 * channels F1-8, Clear and NIR
 *
 * @param readings_buffer Pointer to a buffer of length 10 or more to fill with
 * sensor data
 * @return true: success false: failure
 */
bool Adafruit_AS7341::readAllChannels(uint16_t *readings_buffer) {

  setSMUXLowChannels(true);

  Adafruit_BusIO_Register channel_data_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);

  bool low_success = channel_data_reg.read((uint8_t *)readings_buffer, 12);

  setSMUXLowChannels(false);

  return low_success &&
         channel_data_reg.read((uint8_t *)&readings_buffer[6], 12);
}

/**
 * @brief Take readings for F1-8, Clear and NIR and store them in a buffer
 *
 * @return true: success false: failure
 */
bool Adafruit_AS7341::readAllChannels(void) {
  return readAllChannels(_channel_readings);
}

void Adafruit_AS7341::setSMUXLowChannels(bool f1_f4) {
  enableSpectralMeasurement(false);
  setSMUXCommand(AS7341_SMUX_CMD_WRITE);
  if (f1_f4) {
    setup_F1F4_Clear_NIR();
  } else {
    setup_F5F8_Clear_NIR();
  }
  enableSMUX();
  enableSpectralMeasurement(true);
  while (!getIsDataReady()) {
    delay(1);
  }
}

/**
 * @brief Sets the power state of the sensor
 *
 * @param enable_power true: on false: off
 */
void Adafruit_AS7341::powerEnable(bool enable_power) {
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
  Adafruit_BusIO_RegisterBits pon_en =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 0);
  pon_en.write(enable_power);
}

/**
 * @brief Disable Spectral reading, flicker detection, and power
 *
 * */
void Adafruit_AS7341::disableAll(void) {
  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);

  enable_reg.write(0);
}

/**
 * @brief Enables measurement of spectral data
 *
 * @param enable_measurement true: enabled false: disabled
 * @return true: success false: failure
 */
bool Adafruit_AS7341::enableSpectralMeasurement(bool enable_measurement) {

  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);

  Adafruit_BusIO_RegisterBits spec_enable_bit =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 1);
  return spec_enable_bit.write(enable_measurement);
}

bool Adafruit_AS7341::enableSMUX(void) {

  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
  Adafruit_BusIO_RegisterBits smux_enable_bit =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 4);
  bool success = smux_enable_bit.write(true);

  while (smux_enable_bit.read()) {
    delay(1);
  }

  return success;
}

bool Adafruit_AS7341::setSMUXCommand(as7341_smux_cmd_t command) {
  Adafruit_BusIO_Register cfg6_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CFG6);
  Adafruit_BusIO_RegisterBits smux_command_bits =
      Adafruit_BusIO_RegisterBits(&cfg6_reg, 2, 3);

  return smux_command_bits.write(command);
}

/**
 * @brief Enable control of an attached LED on the LDR pin
 *
 * @param enable_led true: LED enabled false: LED disabled
 * @return true: success false: failure
 */
bool Adafruit_AS7341::enableLED(bool enable_led) {
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CONFIG);
  // Enables control of the LED via the LDR pin
  // 1=control enabled 0 = control disabled
  Adafruit_BusIO_RegisterBits led_sel_bit =
      Adafruit_BusIO_RegisterBits(&config_reg, 1, 3);

  Adafruit_BusIO_Register led_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_LED);
  // turns the LED on or off
  Adafruit_BusIO_RegisterBits led_act_bit =
      Adafruit_BusIO_RegisterBits(&led_reg, 1, 7);

  setBank(true); // Access 0x60-0x74
  return (led_sel_bit.write(enable_led) && led_act_bit.write(enable_led));
}

/**
 * @brief Set the current limit for the LED
 *
 * @param led_current_ma the value to set in milliamps. With a minimum of 4. Any
 * amount under 4 will be rounded up to 4
 *
 * Range is 4mA to 258mA
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setLEDCurrent(uint16_t led_current_ma) {
  // check within permissible range
  if (led_current_ma > 258) {
    return false;
  }
  if (led_current_ma < 4) {
    led_current_ma = 4;
  }
  setBank(true); // Access 0x60 0x74

  Adafruit_BusIO_Register led_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_LED);

  // true = led on , false = off
  Adafruit_BusIO_RegisterBits led_current_bits =
      Adafruit_BusIO_RegisterBits(&led_reg, 7, 0);

  return led_current_bits.write((uint8_t)((led_current_ma - 4) / 2));
}

/**
 * @brief Sets the active register bank
 *
 * The AS7341 uses banks to organize the register making it nescessary to set
 * the correct bank to access a register.
 *

 * @param low **true**:
 * **false**: Set the current bank to allow access to registers with addresses
 of `0x80` and above
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setBank(bool low) {
  Adafruit_BusIO_Register cfg0_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CFG0);
  // register map says shift 3, 0xA9 description says shift 4 with 3 being
  // reserved
  Adafruit_BusIO_RegisterBits bank_bit =
      Adafruit_BusIO_RegisterBits(&cfg0_reg, 1, 4);

  bank_bit.write(low);
}

/**
 * @brief Sets the threshold below which spectral measurements will trigger
 * interrupts when the APERS count is reached
 *
 * @param low_threshold the new threshold
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setLowThreshold(uint16_t low_threshold) {
  Adafruit_BusIO_Register sp_low_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_SP_LOW_TH_L, 2, LSBFIRST);
  return sp_low_threshold_reg.write(low_threshold);
}

/**
 * @brief Returns the current low thighreshold for spectral measurements
 *
 * @return int16_t The current low threshold
 */
uint16_t Adafruit_AS7341::getLowThreshold(void) {
  Adafruit_BusIO_Register sp_low_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_SP_LOW_TH_L, 2, LSBFIRST);
  return sp_low_threshold_reg.read();
}

/**
 * @brief Sets the threshold above which spectral measurements will trigger
 * interrupts when the APERS count is reached
 *
 * @param high_threshold
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setHighThreshold(uint16_t high_threshold) {
  Adafruit_BusIO_Register sp_high_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_SP_HIGH_TH_L, 2, LSBFIRST);
  return sp_high_threshold_reg.write(high_threshold);
}

/**
 * @brief Returns the current high thighreshold for spectral measurements
 *
 * @return int16_t The current high threshold
 */
uint16_t Adafruit_AS7341::getHighThreshold(void) {
  Adafruit_BusIO_Register sp_high_threshold_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_SP_HIGH_TH_L, 2, LSBFIRST);
  return sp_high_threshold_reg.read();
}


/**
 * @brief Enable Interrupts based on spectral measurements
 *
 * @param enable_int true: enable false: disable
 * @return true: success false: falure
 */
bool Adafruit_AS7341::enableSpectralInterrupt(bool enable_int) {
  Adafruit_BusIO_Register int_enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_INTENAB);
  Adafruit_BusIO_RegisterBits sp_int_bit =
      Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 3);
  return sp_int_bit.write(enable_int);
}


bool Adafruit_AS7341::enableSystemInterrupt(bool enable_int) {
  Adafruit_BusIO_Register int_enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_INTENAB);
  Adafruit_BusIO_RegisterBits sien_int_bit =
      Adafruit_BusIO_RegisterBits(&int_enable_reg, 1, 0);
  return sien_int_bit.write(enable_int);
}


// Spectral Interrupt Persistence.
// Defines a filter for the number of consecutive
// occurrences that spectral data must remain outside
// the threshold range between SP_TH_L and
// SP_TH_H before an interrupt is generated. The
// spectral data channel used for the persistence filter
// is set by SP_TH_CHANNEL. Any sample that is
// inside the threshold range resets the counter to 0.

/**
 * @brief Sets the number of times an interrupt threshold must be exceeded
 * before an interrupt is triggered
 *
 * @param cycle_count The number of cycles to trigger an interrupt
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setAPERS(as7341_int_cycle_count_t cycle_count) {
  Adafruit_BusIO_Register pers_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_PERS);
  Adafruit_BusIO_RegisterBits apers_bits =
      Adafruit_BusIO_RegisterBits(&pers_reg, 4, 0);
  return apers_bits.write(cycle_count);
}

/**
 * @brief Set the ADC channel to use for spectral thresholds including
 * interrupts, automatic gain control, and persistance settings
 *
 * @param channel The channel to use for spectral thresholds. Must be a
 * as7341_adc_channel_t **except for** `AS7341_ADC_CHANNEL_5`
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setSpectralThresholdChannel(
    as7341_adc_channel_t channel) {
  if (channel == AS7341_ADC_CHANNEL_5) {
    return false;
  }
  Adafruit_BusIO_Register cfg_12_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CFG12);
  Adafruit_BusIO_RegisterBits spectral_threshold_ch_bits =
      Adafruit_BusIO_RegisterBits(&cfg_12_reg, 3, 0);
  return spectral_threshold_ch_bits.write(channel);
}

/**
 * @brief Returns the current value of the Interupt status register
 *
 * @return uint8_t
 */
uint8_t Adafruit_AS7341::getInterruptStatus(void) {
  Adafruit_BusIO_Register int_status_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
  return (uint8_t)int_status_reg.read();
}

/**
 * @brief Returns the status of the spectral measurement threshold interrupts
 *
 * @return true: interrupt triggered false: interrupt not triggered
 */
bool Adafruit_AS7341::spectralInterruptTriggered(void) {
  Adafruit_BusIO_Register int_status_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);
  Adafruit_BusIO_RegisterBits aint_bit =
      Adafruit_BusIO_RegisterBits(&int_status_reg, 1, 3);

  return aint_bit.read();
}

/**
 * @brief Clear the interrupt status register
 *
 * @return true: success false: failure
 */
bool Adafruit_AS7341::clearInterruptStatus(void) {
  Adafruit_BusIO_Register int_status_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS);

  return int_status_reg.write(0xFF);
}

/**
 * @brief The current state of the spectral measurement interrupt status
 * register
 *
 * @return uint8_t The current status register
 */
uint8_t Adafruit_AS7341::spectralInterruptSource(void) {
  Adafruit_BusIO_Register status3_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS3);

  uint8_t spectral_int_source = status3_reg.read();
  last_spectral_int_source = spectral_int_source;
  return spectral_int_source;
}

/**
 * @brief The status of the low threshold interrupt
 *
 * @return true: low interrupt triggered false: interrupt not triggered
 */
bool Adafruit_AS7341::spectralLowTriggered(void) {
  return (last_spectral_int_source & AS7341_SPECTRAL_INT_LOW_MSK > 0);
}

/**
 * @brief The status of the high threshold interrupt
 *
 * @return true: high interrupt triggered false: interrupt not triggered
 */
bool Adafruit_AS7341::spectralHighTriggered(void) {
  return (last_spectral_int_source & AS7341_SPECTRAL_INT_HIGH_MSK > 0);
}

/**
 * @brief
 *
 * @return true: success false: failure
 */
bool Adafruit_AS7341::getIsDataReady() {
  Adafruit_BusIO_Register status2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_STATUS2);
  Adafruit_BusIO_RegisterBits avalid_bit =
      Adafruit_BusIO_RegisterBits(&status2_reg, 1, 6);

  return avalid_bit.read();
}

/**
 * @brief Configure SMUX for sensors F1-4, Clear and NIR
 *
 */
void Adafruit_AS7341::setup_F1F4_Clear_NIR() {
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
  writeRegister(byte(0x0E), byte(0x00)); // F6/F8 right disabled
  writeRegister(byte(0x0F), byte(0x30)); // F3 right connected to AD2
  writeRegister(byte(0x10), byte(0x01)); // F1 right connected to AD0
  writeRegister(byte(0x11), byte(0x50)); // CLEAR right connected to AD4
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x06)); // NIR connected to ADC5
}

/**
 * @brief Configure SMUX for sensors F5-8, Clear and NIR
 *
 */
void Adafruit_AS7341::setup_F5F8_Clear_NIR() {
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
  writeRegister(
      byte(0x0E),
      byte(0x24)); // F8 right connected to ADC2/ F6 right connected to ADC1
  writeRegister(byte(0x0F), byte(0x00)); // F3 right disabled
  writeRegister(byte(0x10), byte(0x00)); // F1 right disabled
  writeRegister(byte(0x11), byte(0x50)); // CLEAR right connected to AD4
  writeRegister(byte(0x12), byte(0x00)); // Reserved or disabled
  writeRegister(byte(0x13), byte(0x06)); // NIR connected to ADC5
}

/**
 * @brief Configure SMUX for flicker detection
 *
 */
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

// TODO; check for valid values
/**
 * @brief Sets the integration time step count
 *
 * Total integration time will be `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @param atime_value The integration time step count
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setATIME(uint8_t atime_value) {
  Adafruit_BusIO_Register atime_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ATIME);
  return atime_reg.write(atime_value);
}

/**
 * @brief Sets the integration time step size
 *
 * @param astep_value Integration time step size in 2.78 microsecon increments
 * Step size is `(astep_value+1) * 2.78 uS`
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setASTEP(uint16_t astep_value) {
  Adafruit_BusIO_Register astep_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ASTEP_L, 2, LSBFIRST);
  return astep_reg.write(astep_value);
}

/**
 * @brief Set the ADC gain multiplier
 *
 * @param gain_value The gain amount. must be an `as7341_gain_t`
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setGain(as7341_gain_t gain_value) {
  Adafruit_BusIO_Register cfg1_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CFG1);
  return cfg1_reg.write(gain_value);
  // AGAIN bitfield is only[0:4] but the rest is empty
}

/**
 * @brief Detect a flickering light
 * @return The frequency of a detected flicker or 1 if a flicker of
 * unknown frequency is detected
 */
uint16_t Adafruit_AS7341::detectFlickerHz(void) {
  bool isEnabled = true;
  bool isFdmeasReady = false;

  // disable everything; Flicker detect, smux, wait, spectral, power
  disableAll();
  powerEnable(true);

  // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10
  // to CFG6)
  setSMUXCommand(AS7341_SMUX_CMD_WRITE);

  // Write new configuration to all the 20 registers for detecting Flicker
  FDConfig();

  // Start SMUX command
  enableSMUX();

  // Enable SP_EN bit
  enableSpectralMeasurement(true);

  writeRegister(byte(AS7341_ENABLE), byte(0x41));
  delay(500);

  switch (getFlickerDetectStatus()) {
  case 44:
    return 1;
  case 45:
    return 100;
  case 46:
    return 120;
  default:
    return 0;
  }

  powerEnable(true);
}

/**
 * @brief Write a byte to the given register
 *
 * @param addr Register address
 * @param val The value to set the register to
 */
void Adafruit_AS7341::writeRegister(byte addr, byte val) {
  Adafruit_BusIO_Register reg = Adafruit_BusIO_Register(i2c_dev, addr);
  reg.write(val);
}
