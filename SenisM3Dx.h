/*
   SENM3Dx.h - Library to interface Senis SENM3Dx three-axis magnetic field sensor.
   Michael Schneider, 2025
   MIT License
*/

#ifndef SENM3Dx_h
#define SENM3Dx_h

#include <array>
#include <type_traits>
#include <cstddef>

template<typename E>
constexpr std::size_t toIndex(E e) noexcept {
  static_assert(std::is_enum_v<E>, "toIndex requires an enum type");
  return static_cast<std::size_t>(e);
}

enum class Gain : uint8_t {
  G3000 = 0b00,
  G1500 = 0b01,
  G150 = 0b10,
  G15 = 0b11,
  COUNT = 4
};

enum class AxisId : uint8_t {
  X = 0,
  Y = 2,
  Z = 1
};

class SENM3Dx;
class SPIClass;

class SENM3DxAxis {
public:
  SENM3DxAxis(SENM3Dx& ctrl, AxisId axisid, uint8_t addrData, uint8_t addrGain)
    : ctrl_(ctrl),
      id(axisid),
      addrData_(addrData),
      addrGain_(addrGain) {}
  float getValue();
  unsigned int getAdcValue();
  float convertAdcToField(unsigned int adcValue);
  Gain getGain();
  void setGain(Gain gain);
  void activate();
  void deactivate();
  AxisId id;
private:
  std::array<float, 4> adcSens = { 1000.0f, 550.0f, 60.0f, 6.0f };
  std::array<float, 4> adcOffsets = { 32768.0f, 32768.0f, 32768.0f, 32768.0f };
  SENM3Dx& ctrl_;
  uint8_t addrData_;
  uint8_t addrGain_;
};

class SENM3Dx {
public:
  explicit SENM3Dx(SPIClass* spi)
    : spi_(spi),
      spiSettings_(SPI_CLK_RATE, MSBFIRST, SPI_MODE1),
      axes_{
        SENM3DxAxis(*this, AxisId::X, ADC_DATAX, GAIN_CTRL_X),
        SENM3DxAxis(*this, AxisId::Z, ADC_DATAZ, GAIN_CTRL_Z),
        SENM3DxAxis(*this, AxisId::Y, ADC_DATAY, GAIN_CTRL_Y)
      } {
    csPin_ = spi_->pinSS();
  }
  SENM3DxAxis& axis(AxisId id);
  std::array<SENM3DxAxis, 3>& axes();
  float getTemperature();
  float getTemperatureAdcOffset();
  float getTemperatureAdcSensitivity();
  unsigned int getAdcValue(uint8_t adcIndex);
  void setTemperatureAdcOffset(float offset);
  void setTemperatureAdcSensitivity(float sensitivity);
  void updateAdcValues();
  void readRegister(uint8_t* out, uint8_t address, size_t numBytes);
  void writeRegister(const uint8_t* data, uint8_t address, size_t numBytes);
  uint8_t getEEPROMByte(uint8_t address);  // EEPROM reads are always single byte
  uint8_t getStatus();
  std::array<unsigned int, 4> adcValues;

private:
  SPIClass* spi_;
  SPISettings spiSettings_;
  std::array<SENM3DxAxis, 3> axes_;
  uint8_t csPin_;
  uint8_t status_;
  float temperatureSens = 168.48f;
  float temperatureOffset = 20663.1f;
  static constexpr unsigned int SPI_CLK_RATE = 7800000;  // 7.8 MHz
  static constexpr uint8_t EEPROM_ACK = 0xA5;
  // register addresses
  static constexpr uint8_t READ = 0b10000000;          // SENM3DX read command (dynamic register)
  static constexpr uint8_t WRITE = 0b00000000;         // SENM3DX write command (dynamic register)
  static constexpr uint8_t READ_EEPROM = 0b10000001;   // SENM3DX read command (EEPROM)
  static constexpr uint8_t WRITE_EEPROM = 0b00000001;  // SENM3DX write command (EEPROM)
  static constexpr uint8_t REG_STATUS = 0x3F;
  static constexpr uint8_t GAIN_CTRL_X = 0x0E;
  static constexpr uint8_t GAIN_CTRL_Z = 0x0F;
  static constexpr uint8_t GAIN_CTRL_Y = 0x10;
  static constexpr uint8_t CHANNEL_CTRL = 0x09;
  static constexpr uint8_t ADC_DATAX = 0x40;
  static constexpr uint8_t ADC_DATAZ = 0x42;
  static constexpr uint8_t ADC_DATAY = 0x44;
  static constexpr uint8_t ADC_DATAT = 0x46;
};

#endif
