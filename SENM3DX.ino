#include <SPI.h>
#include "SenisM3Dx.h"
SPIClass* hspi = NULL;
SENM3Dx* senm3dx = NULL;

// // sensitivity values (mT/ADU) for gains 3000, 1500, 150, 15
// const float ADC_SENS[3][4] = {
//   {1083.00, 596.00, 65.30, 6.61},
//   {1083.00, 596.70, 65.20, 6.60},
//   {1049.00, 577.50, 62.50, 6.35}
// };
// // offsets (ADU) for gains 3000, 1500, 150, 15
// const float ADC_OFFS[3][4] = {
//   {32786.7, 32720.3, 32602.9, 32600.8},
//   {32689.0, 32687.7, 32646.6, 32648.5},
//   {32530.3, 32596.8, 32633.7, 32648.1}
// };

#include <Wire.h>
#include <Adafruit_SSD1306.h>
// DISPLAY CONFIGURATION
constexpr int SCREEN_WIDTH = 128;  // OLED display width, in pixels
constexpr int SCREEN_HEIGHT = 64;  // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// ESP32 Wroom: 21 (SDA), 22 (SCL)
constexpr int OLED_RESET = -1;        // Reset pin # (or -1 if sharing Arduino reset pin)
constexpr int SCREEN_ADDRESS = 0x3C;  ///< See datasheet for Address
constexpr int LINEHEIGHT = 12;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// state control
unsigned long lastUpdateMillis = 0;
// unsigned long lastDebugUpdateMillis = 0;
enum class DispMode : uint8_t {
  FIELD = 0,
  RAW = 1,
  COUNT = 2
};
DispMode displayMode = DispMode::FIELD;
AxisId selectedAxis = AxisId::X;

// input buttons
struct buttonInput {
  const uint8_t pin;
  unsigned long lastActiveMillis;
};
constexpr uint8_t numInputButtons = 3;
buttonInput buttons[numInputButtons] = {
  { .pin = 4, .lastActiveMillis = 0 },  // axis selector
  { .pin = 5, .lastActiveMillis = 0 },  // gain selector
  { .pin = 2, .lastActiveMillis = 0 }   // mode selector
};

void setup() {
  Serial.begin(9600);

  // default HSPI pins: SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi = new SPIClass(HSPI);
  // start the SPI library:
  hspi->begin();
  // initialize the chip select pin:
  pinMode(hspi->pinSS(), OUTPUT);

  // initialize sensor class
  senm3dx = new SENM3Dx(hspi);

  // Wait for display
  delay(500);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.dim(true);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  // setup input buttons
  for (buttonInput& button : buttons) {
    pinMode(button.pin, INPUT);
  }
}

void loop() {

  const unsigned long now = millis();
  handleButtons();
  
  // const unsigned long nowMicros = micros();
  // updating ADC values takes less than 50 us at 7.8 MHz
  senm3dx->updateAdcValues();

  // const unsigned long lastUpdateDuration = micros() - nowMicros;

  if ((now - lastUpdateMillis) > 100) {
    updateDisplay();
    lastUpdateMillis = now;
  }

  // if ((now - lastDebugUpdateMillis) > 1000) {
  //   Serial.println(lastUpdateDuration);
  //   lastDebugUpdateMillis = now;
  // }

}

void updateDisplay() {
  const char axisLabels[] = "xyz";
  char lineBuffer[32];

  for (SENM3DxAxis& axis : senm3dx->axes()) {
    const uint8_t i = toIndex(axis.id);
    const char isSelected = (axis.id == selectedAxis) ? '>' : ' ';

    switch (displayMode) {
      case DispMode::FIELD:
        snprintf(lineBuffer, sizeof(lineBuffer),
                 "%c M%c: %+7.2f mT G%d",
                 isSelected, axisLabels[i], axis.getValue(), axis.getGain()
                 );
        break;
      case DispMode::RAW:
        snprintf(lineBuffer, sizeof(lineBuffer),
                 "%c M%c: %6d G%d",
                 isSelected, axisLabels[i], axis.getAdcValue(), axis.getGain()
                 );
        break;
      default:
        snprintf(lineBuffer, sizeof(lineBuffer),
                 "unknown mode: %d", toIndex(displayMode));
    }

    display.setCursor(4, i * LINEHEIGHT);
    display.print(lineBuffer);
  }

  snprintf(lineBuffer, sizeof(lineBuffer), "Gain: %7d", senm3dx->axis(selectedAxis).getGain());
  display.setCursor(4, 3 * LINEHEIGHT);
  display.print(lineBuffer);

  snprintf(lineBuffer, sizeof(lineBuffer), "Temp: %7.1f C", senm3dx->getTemperature());
  display.setCursor(4, 4 * LINEHEIGHT);
  display.print(lineBuffer);

  display.display();
}

void handleButtons() {
  // axis selector
  if (checkButton(buttons[0])) {
    selectedAxis = static_cast<AxisId>((toIndex(selectedAxis) + 1) % 3);
  }

  // gain selector
  if (checkButton(buttons[1])) {
    uint8_t gainIndex = toIndex(senm3dx->axis(selectedAxis).getGain());
    gainIndex = (gainIndex + 1) % toIndex(Gain::COUNT);
    senm3dx->axis(selectedAxis).setGain(static_cast<Gain>(gainIndex));
  }

  // mode selector
  if (checkButton(buttons[2])) {
    displayMode = static_cast<DispMode>((toIndex(displayMode) + 1) % toIndex(DispMode::COUNT));
    display.clearDisplay();
  }
}

bool checkButton(buttonInput& button) {
  const unsigned long now = millis();

  if ((now - button.lastActiveMillis) > 500 && digitalRead(button.pin)) {
    button.lastActiveMillis = now;
    return true;
  } else {
    return false;
  }
}