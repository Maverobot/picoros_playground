#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "led_strip_esp32.h"
#include "nvs_flash.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "flashing_leds.h"

#define TAG "JOYSTICK_LED"

// Joystick pins
const gpio_num_t JOY_VRX_GPIO = (gpio_num_t)4;
const gpio_num_t JOY_VRY_GPIO = (gpio_num_t)5;
const gpio_num_t JOY_SW_GPIO = (gpio_num_t)10;

// ADC1 channel mapping for ESP32-S3
#define JOY_VRX_CHANNEL  ADC1_CHANNEL_3   // GPIO4
#define JOY_VRY_CHANNEL  ADC1_CHANNEL_4   // GPIO5

// Dead zone to prevent flicker near center
#define DEAD_ZONE 0.1f

// Normalize ADC 0-4095 → float -1.0 to 1.0 with dead zone
static inline float normalize_axis(int raw)
{
    float norm = ((float)raw / 2047.5f) - 1.0f;  // 0->-1, 2047->0, 4095->1
    if (fabsf(norm) < DEAD_ZONE)
        return 0.0f;
    return norm;
}

// Convert normalized value [-1,1] to LED intensity [0-255]
static inline uint8_t axis_to_color(float norm)
{
    if (norm > 0.0f)
        return (uint8_t)(norm * 255.0f);
    else
        return 0;
}

void joystick_leds()
{
    esp_log_level_set("*", ESP_LOG_INFO); // Show INFO and higher

    // ---------------------------------------
    // LED Init
    // ---------------------------------------
    led_strip_init();
    set_led_color(0, 0, 0);
    blink_rgb(200, 165, 0, 1000);

    // ---------------------------------------
    // NVS Init
    // ---------------------------------------
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ---------------------------------------
    // ADC Init (VRx, VRy)
    // ---------------------------------------
    adc1_config_width(ADC_WIDTH_BIT_12);  // 0‒4095 range
    adc1_config_channel_atten(JOY_VRX_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOY_VRY_CHANNEL, ADC_ATTEN_DB_11);

    // ---------------------------------------
    // SW (button) input with pull-up
    // ---------------------------------------
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_SW_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Normalized joystick LED control started");

    bool last_pressed = false;
    bool enabled = true;

    // ---------------------------------------
    // Main loop
    // ---------------------------------------
    while (true)
    {
        const int raw_x = adc1_get_raw(JOY_VRX_CHANNEL);  // 0–4095
        const int raw_y = adc1_get_raw(JOY_VRY_CHANNEL);  // 0–4095

        const bool pressed = (gpio_get_level(JOY_SW_GPIO) == 0);
        if (pressed && pressed != last_pressed)
        {
            enabled = !enabled;
            ESP_LOGI(TAG, "Joystick LED control %s", enabled ? "ENABLED" : "DISABLED");
        }
        last_pressed = pressed;

        if (!enabled)
        {
            set_led_color(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Normalize joystick to [-1,1]
        const float norm_x = normalize_axis(raw_x);
        const float norm_y = normalize_axis(raw_y);

        // Map to LED colors
        const uint8_t red   = axis_to_color(norm_x);    // X positive → red
        const uint8_t green = axis_to_color(norm_y);    // Y positive → green
        const uint8_t blue  = axis_to_color(-norm_y);   // Y negative → blue

        set_led_color(red, green, blue);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
