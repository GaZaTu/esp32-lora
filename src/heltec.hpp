#pragma once

#include "pin_config.hpp"
#include <Arduino.h>

void setBoardLED(int percent) {
  constexpr uint32_t LED_FREQ = 5000;
  constexpr uint8_t LED_CHAN = 0;
  constexpr uint8_t LED_RES = 8;

  if (percent > 0) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_BOARD_LED, LED_FREQ, LED_RES);
    ledcWrite(PIN_BOARD_LED, percent * 255 / 100);
#else
    ledcSetup(LED_CHAN, LED_FREQ, LED_RES);
    ledcAttachPin(PIN_BOARD_LED, LED_CHAN);
    ledcWrite(LED_CHAN, percent * 255 / 100);
#endif
  } else {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcDetach(PIN_BOARD_LED);
#else
    ledcDetachPin(PIN_BOARD_LED);
#endif
    pinMode(PIN_BOARD_LED, INPUT);
  }
}

void enableDisplay(bool enable = true) {
  if (enable) {
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, LOW);
    delay(5);

    pinMode(PIN_DISPLAY_RST, OUTPUT);
    digitalWrite(PIN_DISPLAY_RST, HIGH);
    delay(1);
    digitalWrite(PIN_DISPLAY_RST, LOW);
    delay(20);
    digitalWrite(PIN_DISPLAY_RST, HIGH);
  } else {
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, HIGH);
  }
}

float getBatteryVoltage() {
  float vbat = analogRead(PIN_VBAT_ADC) / 238.7;
  return vbat;
}

int getBatteryPercentage(float vbat = -1) {
  const float min_voltage = 3.04;
  const float max_voltage = 4.26;
  const uint8_t scaled_voltage[] = {
    254, 242, 230, 227, 223, 219, 215, 213, 210, 207,
    206, 202, 202, 200, 200, 199, 198, 198, 196, 196,
    195, 195, 194, 192, 191, 188, 187, 185, 185, 185,
    183, 182, 180, 179, 178, 175, 175, 174, 172, 171,
    170, 169, 168, 166, 166, 165, 165, 164, 161, 161,
    159, 158, 158, 157, 156, 155, 151, 148, 147, 145,
    143, 142, 140, 140, 136, 132, 130, 130, 129, 126,
    125, 124, 121, 120, 118, 116, 115, 114, 112, 112,
    110, 110, 108, 106, 106, 104, 102, 101, 99, 97,
    94, 90, 81, 80, 76, 73, 66, 52, 32, 7,
  };

  if (vbat == -1) {
    vbat = getBatteryVoltage();
  }
  for (int n = 0; n < sizeof(scaled_voltage); n++) {
    float step = (max_voltage - min_voltage) / 256;
    if (vbat > min_voltage + (step * scaled_voltage[n])) {
      return 100 - n;
    }
  }
  return 0;
}
