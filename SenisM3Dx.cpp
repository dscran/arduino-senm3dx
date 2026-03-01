/*
   SENM3Dx.cpp - Library to interface Senis SENM3Dx three-axis magnetic field sensor.
   Michael Schneider, 2025
   MIT License
*/

#include <Arduino.h>
#include <SPI.h>
#include "SenisM3Dx.h"

// Axis class

void SENM3DxAxis::activate() {
  // TODO
}

void SENM3DxAxis::deactivate() {
  // TODO
}

Gain SENM3DxAxis::getGain() {
  uint8_t buffer[1];
  ctrl_.readRegister(buffer, addrGain_, 1);
  return static_cast<Gain>(buffer[0]);
}

void SENM3DxAxis::setGain(Gain gain) {
  const uint8_t buffer[1] = { static_cast<uint8_t>(gain) };
  ctrl_.writeRegister(buffer, addrGain_, 1);
}

unsigned int SENM3DxAxis::getAdcValue() {
  return ctrl_.adcValues[toIndex(id)];
}

float SENM3DxAxis::convertAdcToField(unsigned int adcValue) {
  const size_t gainIndex = toIndex(getGain());
  return (adcValue - adcOffsets[gainIndex]) / adcSens[gainIndex];
}

float SENM3DxAxis::getValue() {
  return convertAdcToField(getAdcValue());
}


// SENM3Dx class

void SENM3Dx::readRegister(uint8_t* out, uint8_t address, size_t numBytes) {
  /*
  Read <numBytes> consecutive bytes, starting from <address> in dynamic register
  and write them into <out>.
  */
  spi_->beginTransaction(spiSettings_);
  digitalWrite(csPin_, LOW);

  // Transfers are simultaneously bidirectional (full duplex).
  // In addition to the command and address, we send zeros until desired number of bytes read.
  uint8_t buffer[3 + numBytes];
  const uint8_t command[3 + numBytes] = { READ, address };
  spi_->transferBytes(command, buffer, 3 + numBytes);

  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();

  for (int i = 0; i < numBytes; i++) {
    out[i] = buffer[3 + i];
  }
}

void SENM3Dx::updateAdcValues() {
  // read status byte and all 8 ADC data bytes in a single frame.
  const static uint8_t numBytesToRead = 9;
  uint8_t buffer[numBytesToRead];

  readRegister(buffer, REG_STATUS, numBytesToRead);
  status_ = buffer[0];
  for (uint8_t i = 0; i < 4; i++) {
    adcValues[i] = (
      static_cast<uint16_t>(buffer[2 + 2 * i]) << 8) 
      | static_cast<uint16_t>(buffer[1 + 2 * i]
      );
  }
}

void SENM3Dx::writeRegister(const uint8_t* data, uint8_t address, size_t numBytes) {
  /*
  Write data to dynamic register. Writing multiple consecutive bytes is supported.
  */
  spi_->beginTransaction(spiSettings_);
  digitalWrite(csPin_, LOW);

  spi_->transfer(WRITE);
  spi_->transfer(address);

  for (int i = 0; i < numBytes; i++) {
    spi_->transfer(data[i]);
  }

  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();
}

uint8_t SENM3Dx::getEEPROMByte(uint8_t address) {
  // EEPROM reads can only return a single data byte.
  // The number of empty bytes before the data byte is not guaranteed.
  // Instead there's a sentinel acknowledge value (0xA5) that marks the
  // start of the EEPROM data. We wait at most 16 bytes before aborting.

  uint8_t data = 0;

  spi_->beginTransaction(spiSettings_);
  digitalWrite(csPin_, LOW);

  spi_->transfer(READ_EEPROM);
  spi_->transfer(address);

  for (int count = 0; count < 16; count++) {
    if (spi_->transfer(0x00) == EEPROM_ACK) {
      data = spi_->transfer(0x00);
      break;
    }
  }

  digitalWrite(csPin_, HIGH);
  spi_->endTransaction();

  return data;
}

SENM3DxAxis& SENM3Dx::axis(AxisId id) {
  return axes_[toIndex(id)];
}

std::array<SENM3DxAxis, 3>& SENM3Dx::axes() {
  return axes_;
}

float SENM3Dx::getTemperature() {
  return (adcValues[3] - temperatureOffset) / temperatureSens;
}

float SENM3Dx::getTemperatureAdcOffset() {
  return temperatureOffset;
}
float SENM3Dx::getTemperatureAdcSensitivity() {
  return temperatureSens;
}

void SENM3Dx::setTemperatureAdcOffset(float offset) {
  temperatureOffset = offset;
}

void SENM3Dx::setTemperatureAdcSensitivity(float sensitivity) {
  temperatureSens = sensitivity;
}

uint8_t SENM3Dx::getStatus() {
  return status_;
}
