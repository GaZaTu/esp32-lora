#pragma once

#include <Arduino.h>

#if ESP_ARDUINO_VERSION_MAJOR >= 3
#include "driver/temperature_sensor.h"
#else
#include "driver/temp_sensor.h"
#endif

float getESP32Temperature() {
  float result = 0;

  // If temperature for given n below this value,
  // then this is the best measurement we have.
  int cutoffs[5] = {-30, -10, 80, 100, 2500};

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  int range_start[] = {-40, -30, -10, 20, 50};
  int range_end[] = {20, 50, 80, 100, 125};
  temperature_sensor_handle_t temp_handle = NULL;
  for (int n = 0; n < 5; n++) {
    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(range_start[n], range_end[n]);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &result));
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));
    ESP_ERROR_CHECK(temperature_sensor_uninstall(temp_handle));
    if (result <= cutoffs[n]) {
      break;
    }
  }
#else
  // We start with the coldest range, because those temps get spoiled
  // the quickest by heat of processor waking up.
  temp_sensor_dac_offset_t offsets[5] = {
      TSENS_DAC_L4, // (-40°C ~  20°C, err <3°C)
      TSENS_DAC_L3, // (-30°C ~  50°C, err <2°C)
      TSENS_DAC_L2, // (-10°C ~  80°C, err <1°C)
      TSENS_DAC_L1, // ( 20°C ~ 100°C, err <2°C)
      TSENS_DAC_L0  // ( 50°C ~ 125°C, err <3°C)
  };
  for (int n = 0; n < 5; n++) {
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = offsets[n];
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
    temp_sensor_read_celsius(&result);
    temp_sensor_stop();
    if (result <= cutoffs[n]) {
      break;
    }
  }
#endif

  return result;
}
