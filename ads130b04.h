/*

Copyright 2022 Ra Cleave 

This file is part of ADS130B04 LIB.

ADS130B04 LIB is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

ADS130B04 LIB is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with ADS130B04 LIB. If not, see
<https://www.gnu.org/licenses/>.  

*/

#ifndef ADS130B04_H
#define ADS130B04_H

#include "Arduino.h"
//#include "Wire.h"

#define ADS130B04_LIB_VERSION (F("0.1.1"))

#define ADS130B04_ADDR_ID 0x00
#define ADS130B04_ADDR_STATUS 0x01
#define ADS130B04_ADDR_MODE 0x02
#define ADS130B04_ADDR_CLOCK 0x03
#define ADS130B04_ADDR_GAIN 0x04
#define ADS130B04_ADDR_GLOBAL_CHOP_CFG 0x06
#define ADS130B04_ADDR_CH0_CFG 0x09
#define ADS130B04_ADDR_CH1_CFG 0x0E
#define ADS130B04_ADDR_CH2_CFG 0x13
#define ADS130B04_ADDR_CH3_CFG 0x18

#define ADS130B04_OK 0
#define ADS130B04_ERROR -1
#define ADS130B04_ERROR_WRITING_TO_REGISTER -2
#define ADS130B04_INCORRECT_REGISTER -3

/** ADS130B04 read command. Format is "101a aaa annn nnnn", where
    aaaaa is the address of the first register, and nnnnnnn is the
    number of registers to read minus one.
**/
const uint16_t READ = 0b101000000000000;

/** ADS130B04 write command. Format is "011a aaa annn nnnn", where
    aaaaa is the address of the first register, and nnnnnnn is the
    number of registers to write to minus one.
**/
const uint16_t WRITE = 0b0110000000000000;
  

/***************************

Some brief notes from the datasheet (https://www.ti.com/lit/ds/symlink/ads130b04-q1.pdf?ts=1670873352210)
SPI mode 1, so:

 - data clock idles low
 - data are launched/changed on CLK rising edges
 - data are latched/read on CLK falling edges

Full duplex.

Speed = ?

Byte order = ? (prob MSB)

Device commands: NULL, RESET, STANDBY, WAKEUP, LOCK, UNLOCK, RREG, WREG

Modulator overs
000b = 128 (32 kSPS)
001b = 256
010b = 512
011b = 1024
100b = 2048
101b = 4096
110b = 8192
111b = 16384 (250 SPS)

***************************/

class ADS130B04
{
public:
  ADS130B04(uint8_t _chipSelectPin, uint8_t _dataReadyPin, long unsigned int _spiFreq);

  int chipSelectPin;
  int dataReadyPin;
  long int spiFreq;
  const uint8_t wordLength = 3;

  uint8_t enabledChannels = 0x0F;
  uint8_t osr = 0b111;
  uint8_t pwr = 0b10;
  uint8_t extClock = 1;
  
  //SPISettings spiSettings;

  //! Initialise SPI stuff and start device.
  int8_t begin(uint16_t _fs);
    
  //! Get status register
  int8_t status();

  //! Reset device
  int8_t reset();

  //! Set sampling rate.
  int8_t setSPS(uint16_t sps);

  //! Set which channels are enabled.
  int8_t enableChannels(bool ch0, bool ch1, bool ch2, bool ch3);
  int8_t enableChannels(byte _chList);
  
  //! Set resolution.
  int8_t setResolution(uint8_t res);
  
  //! Set device to standby
  int8_t standby();

  //! Wakeup device
  int8_t wakeup();

  //! Lock device
  int8_t lock();
  
  //! Unlock device
  int8_t unlock();
  
  //! Read register
  int8_t read(int16_t* dataPtr);
  
  //! Write to register
  int8_t write();
  
  int8_t writeClockRegister();
  
  int8_t readRegister(byte thisRegister, uint8_t bytesToRead);
  
  int8_t writeRegister(byte thisRegister, uint16_t thisValue);
  
private:
  int value;
  //void doSomethingSecret(void);

  
protected:

  // Start reading the manual.
  
  uint8_t  _config;
  uint8_t  _maxPorts;
  uint8_t  _address;
  uint8_t  _conversionDelay;
  uint8_t  _bitShift;
  uint16_t _gain;
  uint16_t _mode;
  uint16_t _datarate;

};


#endif // ADS130B04_H
