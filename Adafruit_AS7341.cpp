/*!
 *  @file Adafruit_AS7341.cpp
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

  setSMUXLowChannels(true);        // Configure SMUX to read low channels
  enableSpectralMeasurement(true); // Start integration
  delayForData(0);                 // I'll wait for you for all time

  Adafruit_BusIO_Register channel_data_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);

  bool low_success = channel_data_reg.read((uint8_t *)readings_buffer, 12);

  setSMUXLowChannels(false);       // Configure SMUX to read high channels
  enableSpectralMeasurement(true); // Start integration
  delayForData(0);                 // I'll wait for you for all time

  return low_success &&
         channel_data_reg.read((uint8_t *)&readings_buffer[6], 12);
}

/**
 * @brief starts the process of getting readings from all channels without using
 * delays
 *
 * @return true: success false: failure (a bit arbitrary)
 */
bool Adafruit_AS7341::startReading(as7341_channel_group_t channelsOption) {
  _readingState = AS7341_WAITING_START; // Start the measurement please
  _channelsOption = channelsOption;		// Whether to read low, high, both
  checkReadingProgress();               // Call the check function to start it
  return true;
}

/**
 * @brief runs the process of getting readings from all channels without using
 * delays.  Should be called regularly (ie. in loop()) Need to call
 * startReading() to initialise the process Need to call getAllChannels() to
 * transfer the data into an external buffer
 *
 * @return true: reading is complete false: reading is incomplete (or failed)
 */
bool Adafruit_AS7341::checkReadingProgress() {
  if (_readingState == AS7341_WAITING_START) {
	if(_channelsOption == AS7341_CH_LOW || _channelsOption == AS7341_CH_BOTH) {
		setSMUXLowChannels(true);        // Configure SMUX to read low channels
		enableSpectralMeasurement(true); // Start integration
		_readingState = AS7341_WAITING_LOW;
		return false;
	}
	else if(_channelsOption == AS7341_CUSTOM_SMUX) {
		setSMUXCustomChannels();	//Writes your pre-prepared SMUX config to device
		enableSpectralMeasurement(true); // Start integration
		_readingState = AS7341_WAITING_LOW;
		return false;		
	}
	else {		
		for(int i=0;i<6;i++)	//Blank the low channel results
			_channel_readings[i] = 0;
		_readingState = AS7341_WAITING_CONTINUE;
	}
  }

  if(_readingState != AS7341_WAITING_CONTINUE) {
	if (!getIsDataReady() || _readingState == AS7341_WAITING_DONE)
		return false; 
  }

  if (_readingState == AS7341_WAITING_LOW || _readingState == AS7341_WAITING_CONTINUE) {
	if(_channelsOption == AS7341_CH_LOW || _channelsOption == AS7341_CH_BOTH || _channelsOption == AS7341_CUSTOM_SMUX) {
		Adafruit_BusIO_Register channel_data_reg =
			Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);
		channel_data_reg.read((uint8_t *)_channel_readings, 12);
	}
	if(_readingState == AS7341_WAITING_CONTINUE)	//Recover from a skipped LOW
		_readingState = AS7341_WAITING_LOW;
	
	if(_channelsOption == AS7341_CH_HIGH || _channelsOption == AS7341_CH_BOTH) {
		setSMUXLowChannels(false);       // Configure SMUX to read high channels
		enableSpectralMeasurement(true); // Start integration
		_readingState = AS7341_WAITING_HIGH;
		return false;
	}
	else {
		for(int i=6;i<12;i++)	//Blank the high channel results
			_channel_readings[i] = 0;
		_readingState = AS7341_WAITING_CONTINUE;			
	}
  }

  if (_readingState == AS7341_WAITING_HIGH 
		|| _readingState == AS7341_WAITING_CONTINUE) {
    _readingState = AS7341_WAITING_DONE;
    if(_channelsOption == AS7341_CH_HIGH || _channelsOption == AS7341_CH_BOTH) {
		Adafruit_BusIO_Register channel_data_reg =
			Adafruit_BusIO_Register(i2c_dev, AS7341_CH0_DATA_L, 2);
		channel_data_reg.read((uint8_t *)&_channel_readings[6], 12);
	}
    return true;
  }

  return false;
}

/**
 * @brief transfer all the values from the private result buffer into one
 * nominated
 *
 * @param readings_buffer Pointer to a buffer of length 12 (THERE IS NO ERROR
 * CHECKING, YE BE WARNED!)
 *
 * @return true: success false: failure
 */
bool Adafruit_AS7341::getAllChannels(uint16_t *readings_buffer) {
  for (int i = 0; i < 12; i++)
    readings_buffer[i] = _channel_readings[i];
  return true;
}

/**
 * @brief Delay while waiting for data, with option to time out and recover
 *
 * @param waitTime the maximum amount of time to wait
 * @return none
 */
void Adafruit_AS7341::delayForData(int waitTime) {
  if (waitTime == 0) // Wait forever
  {
    while (!getIsDataReady()) {
      delay(1);
    }
    return;
  }
  if (waitTime > 0) // Wait for that many milliseconds
  {
    uint32_t elapsedMillis = 0;
    while (!getIsDataReady() && elapsedMillis < waitTime) {
      delay(1);
      elapsedMillis++;
    }
    return;
  }
  if (waitTime < 0) {
    // For future use?
    return;
  }
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

  int timeOut = 1000; // Arbitrary value, but if it takes 1000 milliseconds then
                      // something is wrong
  int count = 0;
  while (smux_enable_bit.read() && count < timeOut) {
    delay(1);
    count++;
  }
  if (count >= timeOut)
    return false;
  else
    return success;
}

bool Adafruit_AS7341::enableFlickerDetection(bool enable_fd) {

  Adafruit_BusIO_Register enable_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_ENABLE);
  Adafruit_BusIO_RegisterBits fd_enable_bit =
      Adafruit_BusIO_RegisterBits(&enable_reg, 1, 6);
  return fd_enable_bit.write(enable_fd);
}

/**
 * @brief Get the GPIO pin direction setting
 *
 * @return `AS7341_OUTPUT` or `AS7341_INPUT`
 */
as7341_gpio_dir_t Adafruit_AS7341::getGPIODirection(void) {
  Adafruit_BusIO_Register gpio2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
  Adafruit_BusIO_RegisterBits gpio_input_enable =
      Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 2);

  return (as7341_gpio_dir_t)gpio_input_enable.read();
}

/**
 * @brief Set the GPIO pin to be used as an input or output
 *
 * @param gpio_direction The IO direction to set
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setGPIODirection(as7341_gpio_dir_t gpio_direction) {
  Adafruit_BusIO_Register gpio2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
  Adafruit_BusIO_RegisterBits gpio_input_enable =
      Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 2);

  return gpio_input_enable.write(gpio_direction);
}

/**
 * @brief Get the output inversion setting for the GPIO pin
 *
 * @return true: GPIO output inverted false: GPIO output normal
 */
bool Adafruit_AS7341::getGPIOInverted(void) {
  Adafruit_BusIO_Register gpio2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
  Adafruit_BusIO_RegisterBits gpio_output_inverted_bit =
      Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 3);

  return gpio_output_inverted_bit.read();
}

/**
 * @brief Invert the logic of then GPIO pin when used as an output
 *
 * @param gpio_inverted **When true** setting the gpio value to **true will
 * connect** the GPIO pin to ground. When set to **false**, setting the GPIO pin
 * value to **true will disconnect** the GPIO pin from ground
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setGPIOInverted(bool gpio_inverted) {
  Adafruit_BusIO_Register gpio2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
  Adafruit_BusIO_RegisterBits gpio_output_inverted_bit =
      Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 3);

  return gpio_output_inverted_bit.write(gpio_inverted);
}

/**
 * @brief Read the digital level of the GPIO pin, high or low
 *
 * @return true: GPIO pin level is high false: GPIO pin level is low
 */
bool Adafruit_AS7341::getGPIOValue(void) {
  Adafruit_BusIO_Register gpio2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
  Adafruit_BusIO_RegisterBits gpio_input_value_bit =
      Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 0);

  return gpio_input_value_bit.read();
}

/**
 * @brief Set the digital level of the GPIO pin, high or low
 *
 * @param gpio_high The GPIO level to set. Set to true to disconnect the pin
 * from ground. Set to false to connect the gpio pin to ground. This can be used
 * to connect the cathode of an LED to ground to turn it on.
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setGPIOValue(bool gpio_high) {
  Adafruit_BusIO_Register gpio2_reg =
      Adafruit_BusIO_Register(i2c_dev, AS7341_GPIO2);
  Adafruit_BusIO_RegisterBits gpio_output_value_bit =
      Adafruit_BusIO_RegisterBits(&gpio2_reg, 1, 1);

  return gpio_output_value_bit.write(gpio_high);
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
  bool result = led_sel_bit.write(enable_led) && led_act_bit.write(enable_led);
  setBank(false); // Access registers 0x80 and above (default)
  return result;
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

  bool result = led_current_bits.write((uint8_t)((led_current_ma - 4) / 2));
  setBank(false); // Access registers 0x80 and above (default)
  return result;
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

  return bank_bit.write(low);
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

/**
 * @brief Enabled system interrupts
 *
 * @param enable_int Set to true to enable system interrupts
 * @return true: success false: failure
 */
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
  // re-enable power
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

  // Enable flicker detection bit
  writeRegister(byte(AS7341_ENABLE), byte(0x41));
  delay(500); // SF 2020-08-12 Does this really need to be so long?
  uint16_t flicker_status = getFlickerDetectStatus();
  enableFlickerDetection(false);
  switch (flicker_status) {
  case 44:
    return 1;
  case 45:
    return 100;
  case 46:
    return 120;
  default:
    return 0;
  }
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


/**
 * @brief Write the contents of SMUXPixelRegistersLocal[] to the device
 * Suggest avoid calling this externally - it is called at the start of 
 * an integration
 */
bool Adafruit_AS7341::setSMUXCustomChannels() {
	enableSpectralMeasurement(false);
	setSMUXCommand(AS7341_SMUX_CMD_WRITE);
	
	for(byte addr=0;addr<SMUX_PIXEL_REG_NUM;addr++)
		writeRegister(addr,SMUXPixelRegistersLocal[addr]);	
	
	enableSMUX();
	return true;
}

/**
 * @brief Clear SMUXPixelRegistersLocal[], the copy of what will be written to the device 
 * for a custom SMUX configuration
 *
 */
bool Adafruit_AS7341::clearSMUXPixelRegistersLocal() {
	for(byte addr=0;addr<SMUX_PIXEL_REG_NUM;addr++)
		SMUXPixelRegistersLocal[addr] = 0;
	return true;
}

/**
 * @brief Add a connection to the local SMUX map
 * Use the macros already defined in the separate header file to 
 * connect a diode to an ADC
 * Example call: "updateSMUXEntryLocal(F3_LEFT, ADC2);"
 *
 * @param diodePixel 
 * @param numADC
 */
bool Adafruit_AS7341::updateSMUXEntryLocal(byte diodePixel, byte numADC) {
	if(diodePixel >= NUM_PIXELS || numADC > 6)	//Error checking
		return false;
	
	byte writeAddress = pixelAddr[diodePixel];
	byte newBitMask = numADC;
	byte blankingMask = 0b111;
	
	newBitMask = newBitMask << (pixelPos[diodePixel] - 2);	//Shift 0 or 4 places	
	blankingMask = blankingMask << (pixelPos[diodePixel] - 2);	//Shift 0 or 4 places
	blankingMask = ~blankingMask;	//Invert it
	// Serial.println(newBitMask, BIN);
	
	//Update the collection of values for bulk writing later
	SMUXPixelRegistersLocal[writeAddress] &= blankingMask; 	//Blank old value bits
	SMUXPixelRegistersLocal[writeAddress] |= newBitMask; 	//Apply new value
	
	return true;
}

/**
 * @brief Use a 32-bit config word to set all the diodes connected to an ADC channel
 * Bits in the config word correspond to indexes of the diodes per the extra header file
 * WARNING: INDEXES ARE ARBITRARY, AND NOT THE SAME AS PIXEL NUMBERS
 * The config word is intended to allow easy reading of a configuration from a file
 * @param inADC
 * @param wordIn
 */
bool Adafruit_AS7341::setSMUXADCFromConfigWord(byte inADC, uint32_t wordIn) {
	if(inADC > 6)	//Error checking
		return 0;
		
	// Serial.println(wordIn,BIN);	
	uint32_t bitMask = 1;
	for(byte i=0; i<NUM_PIXELS; i++)
	{
		// Serial.println(bitMask,BIN);
		if(wordIn & bitMask)	//That's a bitwiseAND, so only the relevant bit is checked
		{
			updateSMUXEntryLocal(i, inADC);
			// Serial.println(bitMask,BIN);
		}
		bitMask = bitMask << 1;		
	}
	return true;
}

/**
 * @brief Add a diode/pixel to a config word that is being generated
 *
 * @param inputWord (note it's a pointer)
 * @param inputPixel (use a macro, like "F1_LEFT" or "PIXEL3")
 */
bool Adafruit_AS7341::addSMUXPixelToADCword(uint32_t* inputWord, byte inputPixel) {
	if(inputPixel >= NUM_PIXELS)
		return false;
	
	uint32_t bitMask = 1 << inputPixel;
	*inputWord |= bitMask;
	
	return true;
}

/**
 * @brief Remove a diode/pixel from a config word that is being generated
 *
 * @param inputWord (note it's a pointer)
 * @param inputPixel (use a macro, like "F1_LEFT" or "PIXEL3")
 */
bool Adafruit_AS7341::removeSMUXPixelFromADCword(uint32_t* inputWord, byte inputPixel) {
	if(inputPixel >= NUM_PIXELS)
		return false;
	
	uint32_t bitMask = 1 << inputPixel;
	bitMask = ~bitMask;	//Invert it for removing
	*inputWord &= bitMask;
	
	return true;
}

/**
 * @brief Generate a config word from the local config for an ADC channel
 *
 * @param inputPixel (use a macro, like "ADC0")
 */
uint32_t Adafruit_AS7341::getSMUXConfigWordForADC(byte inADC) {
	if(inADC > 6)	//Error checking
		return 0;
		
	byte addr = 0;
	byte pos = 0;
	byte bitMask = 0;
	byte addrContents = 0;
	uint32_t newConfigWord = 0;
	for(byte i=0; i<NUM_PIXELS; i++) {
		addr = pixelAddr[i];
		pos = pixelPos[i];
		addrContents = SMUXPixelRegistersLocal[addr];
		
		bitMask = inADC;
		// bitMask = bitMask << (pixelPos[i] - 2);	//Shift 0 or 4 places	
		
		if(pixelPos[i] == 2) {
			addrContents = addrContents << 4;	//Remove other value, if present
			bitMask = bitMask << 4;				//Align with new addrContents
		}
		if(pixelPos[i] == 6) {
			addrContents = addrContents >> 4;	//Remove other value, if present
			// bitMask = bitMask  4;
		}
		if(addrContents == bitMask)
			addSMUXPixelToADCword(&newConfigWord,i);		
	}
	return newConfigWord;	
}

/**
 * @brief Debugging function, prints the local custom SMUX configuration
 *
 * @param inputPixel (use a macro, like "ADC0")
 */
void Adafruit_AS7341::printLocalSMUXConfig() {
	Serial.println("Local SMUX Config:");
	for(byte addr=0;addr<SMUX_PIXEL_REG_NUM;addr++) {
		Serial.print("Address 0x");
		Serial.print(addr, HEX);
		Serial.print(":\t");
		printBits(SMUXPixelRegistersLocal[addr],8);	
		Serial.print(" (0x");
		Serial.print(SMUXPixelRegistersLocal[addr],HEX);	
		Serial.println(")");		
	}
}


/**
 * @brief Helper function
 *
 * Adapted from here: http://engineeringnotes.blogspot.com/2016/10/printbits-routine.html
 * prints N-bit integer in this form: 0000000000000000
 * works for 4 - 32 bits
 * @param inputPixel (use a macro, like "ADC0")
*/
void Adafruit_AS7341::printBits(long int n, int numBits) {
//  byte numBits = 32;  // 2^numBits must be big enough to include the number n
  char b;
  char c = ' ';   // delimiter character
  for (byte i = 0; i < numBits; i++) {
    // shift 1 and mask to identify each bit value
    b = (n & (1 << (numBits - 1 - i))) > 0 ? '1' : '0'; // slightly faster to print chars than ints (saves conversion)
    Serial.print(b);
//    if (i < (numBits - 1) && ((numBits-i - 1) % 4 == 0 )) Serial.print(c); // print a separator at every 4 bits
  }
}
	