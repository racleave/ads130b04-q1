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

                                                                                      
#include <SPI.h>
#include "ads130b04-q1.h"

//! ADS130B04 library class
/*! A class library to setup and read data from the ADS130B04 four
  channel 16bit analog to digital converter.
*/
ADS130B04Q1::ADS130B04Q1(uint8_t _chipSelectPin,
                     uint8_t _dataReadyPin,
                     long unsigned int _spiFreq) {

  dataReadyPin = _dataReadyPin;
  chipSelectPin = _chipSelectPin;
  spiFreq = _spiFreq;
  
  // Initialize the  data ready and chip select pins:
  pinMode(chipSelectPin, OUTPUT);
  pinMode(dataReadyPin, INPUT_PULLUP);
 
}

/*! Initialise sensor. Do a reset, set clock to correct fs.
*/
int8_t ADS130B04Q1::begin(uint16_t _fs) {

  int8_t result = ADS130B04Q1_OK;
  
  // give the sensor time to set up:
  //delay(100);

  // Not working, but possibly not needed here anyway.
  //if (reset() != 0)
  //  Serial.println("Something went wrong resetting!");

  result = wakeup();
  Serial.print("Waking up: ");Serial.println(result);


  //if (standby() != 0)
  //  Serial.println("Something went wrong setting STANDBY!");
  //reset();

  Serial.println("Setting SPS");
  result = setSPS(_fs);
  Serial.print("SPS set: ");Serial.println(result);
  
  Serial.println("Getting status");
  status();
  Serial.println("Getting status");
  status();

  if (wakeup() != 0)
    Serial.println("Something went wrong waking up!");

  Serial.println("Getting status");
  status();
  Serial.println("Getting status");
  status();
  
  return 0;
}

/*! 

Options are 250, 500, 1000, 2000, 4000, 8000, 16000, 32000 Hz. If
*sps* is not one of these then the lowest rate is used.
*/
int8_t ADS130B04Q1::setSPS(uint16_t sps) 
{
  if (sps == 32000) osr = 0b000;
  else if (sps == 16000) osr = 0b001;
  else if (sps == 8000) osr = 0b010;
  else if (sps == 4000) osr = 0b011;
  else if (sps == 2000) osr = 0b100;
  else if (sps == 1000) osr = 0b101;
  else if (sps == 500) osr = 0b110;
  else osr = 0b111; // 250 Hz

  return writeClockRegister();
}

/*! Set the resolution of ADC, whihc also affects power consumption

  - 00b = Very-low power

  - 01b = Low power

  - 10b = High resolution

  - 11b = High resolution

*/
int8_t ADS130B04Q1::setResolution(uint8_t res) 
{
  if (res < 5) {
      pwr = res;
      return writeClockRegister();
    }
  else {
    return ADS130B04Q1_ERROR;
    }
}

/*! Set the channels that will be acquired. Not sure whether this
    saves power or not (zeros are sent for the channels which are not
    set).

*/
int8_t ADS130B04Q1::enableChannels(bool ch0, bool ch1, bool ch2, bool ch3) 
{
  
  byte channelMask = 0x00;
  if (ch0) channelMask |= 0b0001;
  if (ch1) channelMask |= 0b0010;
  if (ch2) channelMask |= 0b0100;
  if (ch3) channelMask |= 0b1000;
  Serial.println(channelMask, BIN);
  enabledChannels = channelMask & 0x0F;
  return writeClockRegister();
}

/*! Set the channels that will be acquired. Not sure whether this
    saves power or not (zeros are sent for the channels which are not
    set).

    Order is Ch3, Ch2, Ch1, Ch0, sent as a byte array (only least
    singificant nibble is used).:

*/
int8_t ADS130B04Q1::enableChannels(byte channelMask) 
{
  enabledChannels = channelMask & 0x0F;
  writeClockRegister();
}


/*! First nibble must be zero, second is which channels are enabled. 2
 byte bit 7 is CLK_SEL, bits 4-2 are OSR, bits 1-0 are
 resolution. Maybe make this be called by functions set_fs(),
 set_res(), enable_channels(0b1111).

*/
int8_t ADS130B04Q1::writeClockRegister() 
{

  int8_t res = ADS130B04Q1_OK;
  
  uint16_t clockRegData = (enabledChannels << 8)
    | (clkSel << 7) | (osr << 2) | pwr;
  Serial.print("Setting clock reg: ");    Serial.println(clockRegData, BIN);
  
  // 0b0000110110011110
  if (writeRegister(ADS130B04Q1_ADDR_CLOCK, clockRegData) == 0) {
    Serial.println("Clock has been set.");
  }
  else {
    Serial.println("Something went wrong writing to the clock register!");
    res = ADS130B04Q1_ERROR;
  }

  return res;
}


/*int ADS130B04Q1::setClock(bool internalClock) {

  return 0;
  }*/


/*! Status is returned with data. Send zeroes in first two words of
    frame and this will back the next frame contains the status in
    first two words and the data in the following four.
*/
int8_t ADS130B04Q1::status() {
  //Serial.println("Checking ADS130B04Q1 status");

  //Serial.println("Expecting something...");
  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);

  byte tmp = 0;
  //int16_t val = 0;
  // 24 bit words - need 6 words, so 18 bytes.
  Serial.print("Status:");
  for (int w = 0; w < 6; w++) {
    //val = 0;
    for (int b = 0; b < wordLength; b++) {
      // The 
      tmp = SPI.transfer(0x00);
      //if (b == 0) val = tmp << 8;
      //else if (b == 1) val = val | tmp;
      if (b < 2) {
        Serial.print(" "); Serial.print(tmp, HEX);
      }
    }
    Serial.print("|");
  }
  Serial.println();
  
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  
  return 0;
}

/*! This assumes that the previous command was either a status() or
    read(), each of which should mean that the current incoming frame
    contains the status and the data.

    Note that data comes through in order 0-3.

    At the moment all four channels are returned even if only some are
    enabled.
*/
int8_t ADS130B04Q1::read(int16_t* data) {

  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);

  byte tmp = 0;
  int16_t val;
  // 24 bit words - need 6 words, so 18 bytes. Or 12?
  for (int w = 0; w < 6; w++) {
    val = 0;
    for (int b = 0; b < wordLength; b++) {
      tmp = SPI.transfer(0x00);
      if (b == 0) val = tmp << 8;
      else if (b == 1) val = val | tmp;
      if ( (w > 0) && (w < 5) ) {
        data[w-1] = val;
      }
    }
  }

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();
  
  return 0;
}

/*! Reset device. A pause is enforced after which the reply is checked
    to see that the reset actually occurred.

   There is something wrong with reset...to be sorted!

*/
int8_t ADS130B04Q1::reset() {
  Serial.println("Resetting ADS130B04Q1");
  Serial.println("...expecting 65364...");
  
  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(0x00);
  SPI.transfer(0b00010001);
  SPI.transfer(0x00); // Padding.
  
  // Need 4 more words for this to be latched.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  delay(5);
  
  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);
  // Then let's check that we actually wrote something. Need to get 6
  // more words, but we only care about the first one.
  byte b1 = 0x00;
  byte b2 = 0x00;
  b1 = SPI.transfer(0x00);
  b2 = SPI.transfer(0x00);
  SPI.transfer(0x00); // The padding.
  // And then the other 4 words which we don't care about.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  int result = ADS130B04Q1_OK;
  
  if (b1 != 0xFF) {
    result = ADS130B04Q1_ERROR;
    Serial.print("Umm ");
    Serial.print(b1, BIN); Serial.print(" "); Serial.println(b2, BIN);
  }
  else {
    if (b2 != 0b01010100) {
      Serial.print("Yikes");
      Serial.println(b2, BIN);
      
      result = ADS130B04Q1_ERROR;
    }
  }
  
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();


  return result;
}

/*! Set to standby. 
*/
int8_t ADS130B04Q1::standby() {
  //Serial.println("Setting ADS130B04Q1 to standby");

  //Serial.println("Sending 0b 0000 0000 0010 0010, expecting back 34 (0b 0000 0000 0010 0010)");
  
  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);
  
  SPI.transfer(0x00);
  SPI.transfer(0b00100010);
  SPI.transfer(0x00); // Padding

  // Need 4 more words to clear this message.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);
  
  // Then let's check that we actually wrote something. Need to get 6
  // more words, but we only care about the first one.
  byte b1 = 0x00;
  byte b2 = 0x00;
  b1 = SPI.transfer(0x00);
  b2 = SPI.transfer(0x00);
  SPI.transfer(0x00); // The padding.
  // And then the other 4 words which we don't care about.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  int result = ADS130B04Q1_OK;
  
  if (b1 != 0x00) {
    //Serial.print("Umm");
    //Serial.println(b1, BIN);
    result = ADS130B04Q1_ERROR;
  }
  else {
    if (b2 != 0b00100010) {
      //Serial.print("Yikes");
      //Serial.println(b2, BIN);
      result = ADS130B04Q1_ERROR;
    }
  }
  
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  return result;
}

/*! Wakeup the device, checking after command is sent to check
    everything is OK.

    Note that pause between comms is not needed, and end/start
    transaction is not needed. But setting ship low and high is (in
    order to get the device to latch the command?).

*/
int8_t ADS130B04Q1::wakeup() {

  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(0x00);
  SPI.transfer(0b00110011);
  SPI.transfer(0x00); // Padding

  // And then the other 4 words which we don't care about.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  digitalWrite(chipSelectPin, HIGH);
  //SPI.endTransaction();
  
  //delay(2);
  
  //SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);
  // Then let's check that we actually wrote something. Need to get 6
  // more words, but we only care about the first one.
  byte b1 = 0x00;
  byte b2 = 0x00;
  b1 = SPI.transfer(0x00);
  b2 = SPI.transfer(0x00);
  SPI.transfer(0x00); // The padding.
  // And then the other 4 words which we don't care about.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  int result = ADS130B04Q1_OK;
  
  if (b1 != 0x00) {
    Serial.print("Umm");
    Serial.println(b1, BIN);
    result = ADS130B04Q1_ERROR;
  }
  else {
    if (b2 != 0b00110011) {
      Serial.print("Yikes");
      Serial.println(b2, BIN);
      
      result = ADS130B04Q1_ERROR;
    }
  }
  
  digitalWrite(chipSelectPin, HIGH);
  SPI.endTransaction();

  return result;
}

/*! Lock the device.
*/
int8_t ADS130B04Q1::lock() {
  Serial.println("Locking ADS130B04Q1");

  return 0;
}
  
//! Unlock device
int8_t ADS130B04Q1::unlock() {
  Serial.println("Unlocking ADS130B04Q1");

  return 0;
}

// Read from register. THIS WILL WORK WITH UP TO 2 BYTES.
int8_t ADS130B04Q1::readRegister(byte thisRegister, uint8_t bytesToRead) {

  byte inByte = 0;           // incoming byte from the SPI

  unsigned int result = ADS130B04Q1_OK;   // result to return

  Serial.print(thisRegister, BIN);
  Serial.print("\t");

  //word dataToSend = (word)thisRegister | READ;
  uint16_t dataToSend = ((uint16_t)thisRegister << 7) | READ;
  //uint16_t dataToSend = READ;
  
  Serial.println(thisRegister, BIN);

  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);

  Serial.println(dataToSend);
  
  // send the device the register you want to read:
  uint16_t result16 = SPI.transfer16(dataToSend);

  Serial.println(result16);

  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);

  // return the result:

  return result;
}

/*! Write to a single register.
*/
int8_t ADS130B04Q1::writeRegister(byte thisRegister, uint16_t thisValue) {

  // Combine the register address and the command into two bytes.
  uint16_t wRegInfo = ((uint16_t)(thisRegister << 7)) | WRITE;

  // Take the chip select low to select the device:
  SPI.beginTransaction(SPISettings(spiFreq, MSBFIRST, SPI_MODE1));
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer((byte)(wRegInfo >> 8)); // Send WREG info
  SPI.transfer((byte)(wRegInfo)); // Send WREG info
  SPI.transfer((byte)(0x00)); // LSB padding
  
  SPI.transfer((byte)(thisValue >> 8)); // Send data
  SPI.transfer((byte)(thisValue)); // Send data
  SPI.transfer((byte)(0x00)); // LSB padding

  // Need 4 more words:
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  // This seems to be working without stopping transmission and
  // restarting, or even cycling of CS pin. So not sure why the other
  // routines seem to need a pause...?
  
  // Then let's check that we actually wrote something. Need to get 6
  // more words, but we only care about the first one.
  byte b1 = 0x00;
  byte b2 = 0x00;
  b1 = SPI.transfer(0x00);
  b2 = SPI.transfer(0x00);
  SPI.transfer(0x00); // The padding.
  // And then the other 4 words which we don't care about.
  for (int i = 0; i < 4*3; i++) {
    SPI.transfer(0x00);
  }

  int result = ADS130B04Q1_OK;
  
  if ((b1 >> 5) != 0b010) {
    result = ADS130B04Q1_ERROR_WRITING_TO_REGISTER;
  }
  else {
    byte regAddr =  ((b1 << 1) | (b2 >> 7)) & 0b00111111;
    if (regAddr != thisRegister)
      result = ADS130B04Q1_INCORRECT_REGISTER;
  }
  
  digitalWrite(chipSelectPin, HIGH);

  SPI.endTransaction();
  
  return result;
  
}
