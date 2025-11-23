#pragma once

#include <math.h>  // For fmod() and M_PI
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "led_strip_esp32.h"
#include "nvs_flash.h"
#include "flashing_leds.h"

void flashing_leds()
{
  led_strip_init();
  set_led_color(0, 0, 0);        // Turn off LED at start
  blink_rgb(200, 165, 0, 1000);  // Blink orange to signal start of app_main

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  const float scale = 0.2f;

  while (true)
  {
    set_led_color(0, 0, 255*scale);  // Blue
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_led_color(255*scale, 0, 0);  // Red
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_led_color(0, 255*scale, 0);  // Green
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
