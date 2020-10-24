/*!
 *  @file Adafruit_AS7341_SMUX.h

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
 *  
 *
 * 	@section license License
 *
 * 	
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

/*
  SMUX lookup values transferred from datasheet by Stephen Fordyce, 23/10/2020
  Refer to document "AS7341_AN000666_1-00 - SMUX Config"

  Development environment specifics: Arduino IDE 1.8.12

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef _ADAFRUIT_AS7341_SMUX_H
#define _ADAFRUIT_AS7341_SMUX_H

#define SMUX_PIXEL_REG_NUM 0x14	//13+1, because 0 isn't used

//Mapping of SMUX pixel names to arbitrary index numbers
#define PIXEL1 0
#define PIXEL2 1
#define PIXEL7 2
#define PIXEL8 3
#define PIXEL11 4
#define PIXEL10 5
#define PIXEL13 6
#define PIXEL14 7
#define PIXEL17 8
#define PIXEL19 9
#define PIXEL20 10
#define PIXEL25 11
#define PIXEL26 12
#define PIXEL29 13
#define PIXEL28 14
#define PIXEL31 15
#define PIXEL33 16
#define PIXEL32 17
#define PIXEL35 18
#define PIXEL34 19
#define PIXEL37 20
#define PIXEL39 21
#define PIXEL38 22

#define NUM_PIXELS 23
///////////////////////////////////////

//Mapping of SMUX pixel names to numbers (in datasheet order)
//This order is also how the arbitrary pixel indexes are assigned
#define FX_FLICKER PIXEL39
#define F1_LEFT PIXEL2 
#define F3_LEFT PIXEL1 
#define F5_RIGHT PIXEL19 
#define F7_RIGHT PIXEL20 
#define F6_LEFT PIXEL8 
#define F8_LEFT PIXEL7 
#define F2_RIGHT PIXEL25 
#define F4_RIGHT PIXEL26 
#define F4_LEFT PIXEL11 
#define F2_LEFT PIXEL10 
#define F8_RIGHT PIXEL28 
#define F6_RIGHT PIXEL29 
#define F7_LEFT PIXEL14 
#define F5_LEFT PIXEL13 
#define F3_RIGHT PIXEL31 
#define F1_RIGHT PIXEL32 
#define FX_CLEAR_LEFT PIXEL17 
#define FX_NIR PIXEL38 
#define FX_CLEAR_RIGHT PIXEL35 
#define FX_EXT_IN_GPIO PIXEL33 
#define FX_EXT_IN_INT PIXEL34 
#define RX_DARK PIXEL37 
///////////////////////////////////////


//Mapping of SMUX pixel numbers to addresses
#define PIX1_ADDR 0x00
#define PIX2_ADDR 0x01
//#define UNUSED_A_ADDR 0x02
#define PIX7_ADDR 0x03
#define PIX8_ADDR 0x04
#define PIX11_ADDR 0x05
#define PIX10_ADDR 0x05
#define PIX13_ADDR 0x06
#define PIX14_ADDR 0x07
#define PIX17_ADDR 0x08
#define PIX19_ADDR 0x09
#define PIX20_ADDR 0x0A
//#define UNUSED_B_ADDR 0x0B
#define PIX25_ADDR 0x0C
#define PIX26_ADDR 0x0D
#define PIX29_ADDR 0x0E
#define PIX28_ADDR 0x0E
#define PIX31_ADDR 0x0F
#define PIX33_ADDR 0x10
#define PIX32_ADDR 0x10
#define PIX35_ADDR 0x11
#define PIX34_ADDR 0x11
#define PIX37_ADDR 0x12
#define PIX39_ADDR 0x13
#define PIX38_ADDR 0x13
///////////////////////////////////////

//Mapping of SMUX pixel numbers to address bits
#define PIX1_POS 6
#define PIX2_POS 2
// #define UNUSED_A_POS 2
#define PIX7_POS 6
#define PIX8_POS 2
#define PIX11_POS 6
#define PIX10_POS 2
#define PIX13_POS 6
#define PIX14_POS 2
#define PIX17_POS 6
#define PIX19_POS 6
#define PIX20_POS 2
// #define UNUSED_B_POS 2
#define PIX25_POS 6
#define PIX26_POS 2
#define PIX29_POS 6
#define PIX28_POS 2
#define PIX31_POS 6
#define PIX33_POS 6
#define PIX32_POS 2
#define PIX35_POS 6
#define PIX34_POS 2
#define PIX37_POS 6
#define PIX39_POS 6
#define PIX38_POS 2
///////////////////////////////////////

//Definition of SMUX ADC numbers
#define ADC_DISABLED 0
#define ADC0 1
#define ADC1 2
#define ADC2 3
#define ADC3 4
#define ADC4 5
#define ADC5 6
///////////////////////////////////////

//Some handy config words
#define CWORD_NONE 	  0b00000000000000000000000000000000
#define CWORD_BOTH_F1 0b00000000000000100000000000000010
#define CWORD_BOTH_F2 0b00000000000000000000100000100000
#define CWORD_BOTH_F3 0b00000000000000001000000000000001
#define CWORD_BOTH_F4 0b00000000000000000001000000010000
#define CWORD_BOTH_F5 0b00000000000000000000001001000000
#define CWORD_BOTH_F6 0b00000000000000000010000000001000
#define CWORD_BOTH_F7 0b00000000000000000000010010000000
#define CWORD_BOTH_F8 0b00000000000000000100000000000100
#define CWORD_BOTH_CLEAR 0b00000000000001000000000100000000
#define CWORD_NIR 	  0b00000000010000000000000000000000
///////////////////////////////////////

//Lookup table for SMUX pixel address
//Index is SMUX diode, ie. F3_LEFT or PIXEL1)
const byte pixelAddr[NUM_PIXELS] = {	
	 PIX1_ADDR,
	 PIX2_ADDR,
	 PIX7_ADDR,
	 PIX8_ADDR,
	 PIX11_ADDR,
	 PIX10_ADDR,
	 PIX13_ADDR,
	 PIX14_ADDR,
	 PIX17_ADDR,
	 PIX19_ADDR,
	 PIX20_ADDR,
	 PIX25_ADDR,
	 PIX26_ADDR,
	 PIX29_ADDR,
	 PIX28_ADDR,
	 PIX31_ADDR,
	 PIX33_ADDR,
	 PIX32_ADDR,
	 PIX35_ADDR,
	 PIX34_ADDR,
	 PIX37_ADDR,
	 PIX39_ADDR,
	 PIX38_ADDR
	};

//Lookup table for SMUX pixel address positions within the address byte
//Index is SMUX diode, ie. F3_LEFT or PIXEL1)
const byte pixelPos[NUM_PIXELS] = {
	 PIX1_POS,
	 PIX2_POS,
	 PIX7_POS,
	 PIX8_POS,
	 PIX11_POS,
	 PIX10_POS,
	 PIX13_POS,
	 PIX14_POS,
	 PIX17_POS,
	 PIX19_POS,
	 PIX20_POS,
	 PIX25_POS,
	 PIX26_POS,
	 PIX29_POS,
	 PIX28_POS,
	 PIX31_POS,
	 PIX33_POS,
	 PIX32_POS,
	 PIX35_POS,
	 PIX34_POS,
	 PIX37_POS,
	 PIX39_POS,
	 PIX38_POS,
	};



#endif

