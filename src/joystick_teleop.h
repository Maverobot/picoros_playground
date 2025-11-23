#pragma once

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>  // for gettimeofday

#include "esp_log.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "led_strip_esp32.h"
#include "nvs_flash.h"

#include "picoros.h"
#include "picoserdes.h"
#include "my_ros_message_types.h"
#include "wifi_time.h"

namespace joystick_teleop
{

// Communication mode (match embedded_mobile_base)
#define MODE "client"
// Router address / UART transport (match embedded_mobile_base)
#define ROUTER_ADDRESS "serial/UART_0#baudrate=460800"

#define TAG "JOYSTICK_TELEOP"

// Joystick pins (update if needed for your board)
static const gpio_num_t JOY_VRX_GPIO = (gpio_num_t)4;
static const gpio_num_t JOY_VRY_GPIO = (gpio_num_t)5;
static const gpio_num_t JOY_SW_GPIO  = (gpio_num_t)10;

// ADC1 channel mapping for ESP32-S3
#define JOY_VRX_CHANNEL  ADC1_CHANNEL_3   // GPIO4
#define JOY_VRY_CHANNEL  ADC1_CHANNEL_4   // GPIO5

// Dead zone to prevent flicker / jitter near center
#define DEAD_ZONE 0.1f

// Scaling from joystick normalized units [-1,1] to velocities
// Tune these for your robot
#define MAX_LINEAR_VEL_MPS   0.5f     // forward/backward, m/s
#define MAX_ANGULAR_VEL_RAD  1.5f     // yaw, rad/s

// --- Pico-ROS node and publisher (pattern copied from embedded_mobile_base) ---

// Node (visible on host with `ros2 node list`)
picoros_node_t jt_node = {
  .name = "wbot_joystick_teleop_node",
};

// Publisher for TwistStamped on /cmd_vel
picoros_publisher_t jt_pub_cmd_vel = {
  .topic =
    {
      .name = "cmd_vel",                         // ROS topic: /cmd_vel
      .type = ROSTYPE_NAME(ros_TwistStamped),
      .rihs_hash = ROSTYPE_HASH(ros_TwistStamped),
    },
};

// Buffer for serialization
static uint8_t jt_pub_buf[256];

// Static frame_id buffer for TwistStamped header
static char jt_frame_id_buf[] = "base_link";

// --- Helpers ---------------------------------------------------------------

// A helper function to get milliseconds (same as embedded_mobile_base)
static inline uint32_t jt_get_milliseconds()
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// Normalize ADC 0-4095 → float -1.0 to 1.0 with dead zone
static inline float jt_normalize_axis(int raw)
{
  // 0 -> -1, 2047~2048 -> 0, 4095 -> 1
  float norm = ((float)raw / 2047.5f) - 1.0f;
  if (fabsf(norm) < DEAD_ZONE)
    return 0.0f;
  if (norm > 1.0f)
    norm = 1.0f;
  if (norm < -1.0f)
    norm = -1.0f;
  return norm;
}

// Convert joystick axes to linear / angular velocity
static inline void jt_axes_to_cmd(float norm_x, float norm_y, float * linear_x, float * angular_z)
{
  // Convention:
  //  - X positive (pushed forward) -> forward linear.x
  //  - Y positive (left)           -> positive angular.z (turn left)
  *linear_x  = norm_x * MAX_LINEAR_VEL_MPS;

  if (norm_x < 0.0f)
  {
    // When going backward, invert turning direction for more intuitive control
    norm_y = -norm_y;
  }

  *angular_z = -norm_y * MAX_ANGULAR_VEL_RAD;
}

// --- Hardware init (mirrors embedded_mobile_base sequence) -----------------

static void jt_hw_init()
{
  // LED init + startup blink (same style as embedded_mobile_base)
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

  set_led_color(0, 0, 255);  // Solid blue to signal NVS init complete
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Obtain correct time via WiFi before starting Pico-ROS
  set_led_color(0, 0, 255);  // Keep blue while getting time
  obtain_time_via_wifi();

  set_led_color(255, 0, 0);  // Solid red to signal start of RMW init

  // ADC init
  adc1_config_width(ADC_WIDTH_BIT_12);  // 0‒4095
  adc1_config_channel_atten(JOY_VRX_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(JOY_VRY_CHANNEL, ADC_ATTEN_DB_11);

  // Joystick button as input with pull-up
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << JOY_SW_GPIO),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);
}

// --- Pico-ROS init (copied from embedded_mobile_base and adapted) ----------

static void jt_ros_init()
{
  // Initialize Pico ROS interface (same pattern as embedded_mobile_base)
  picoros_interface_t ifx = {
    .mode = MODE,
    .locator = ROUTER_ADDRESS,
  };

  ESP_LOGI(
    jt_node.name,
    "Starting pico-ros interface:[%s] on router address:[%s]\n",
    ifx.mode,
    ifx.locator);

  while (picoros_interface_init(&ifx) == PICOROS_NOT_READY)
  {
    printf("Waiting RMW init...\n");
    // Blink LED Red to signal we are waiting for RMW router
    blink_rgb(255, 0, 0, 1000);
    z_sleep_ms(1);
  }

  ESP_LOGI(
    jt_node.name,
    "Starting Pico-ROS node:[%s] domain:[%ld]\n",
    jt_node.name,
    jt_node.domain_id);
  picoros_node_init(&jt_node);

  ESP_LOGI(jt_node.name, "Declaring publisher on [%s]\n", jt_pub_cmd_vel.topic.name);
  picoros_publisher_declare(&jt_node, &jt_pub_cmd_vel);

  // After ROS is fully up, set LED to green to indicate ready for teleop
  set_led_color(0, 255, 0);
}

// --- Main publishing task (similar to publish_joint_state) -----------------

static void jt_publish_cmd_vel_task(void * pvParameters)
{
  (void)pvParameters;

  bool last_pressed = false;
  bool enabled      = true;

  while (true)
  {
    // Read raw joystick axes
    const int raw_x = adc1_get_raw(JOY_VRX_CHANNEL);  // 0–4095
    const int raw_y = adc1_get_raw(JOY_VRY_CHANNEL);  // 0–4095

    // Button toggles enable/disable
    const bool pressed = (gpio_get_level(JOY_SW_GPIO) == 0);
    if (pressed && pressed != last_pressed)
    {
      enabled = !enabled;
      ESP_LOGI(TAG, "Joystick teleop %s", enabled ? "ENABLED" : "DISABLED");
    }
    last_pressed = pressed;

    float norm_x    = 0.0f;
    float norm_y    = 0.0f;
    float linear_x  = 0.0f;
    float angular_z = 0.0f;

    if (enabled)
    {
      // Normalize joystick to [-1,1]
      norm_x = jt_normalize_axis(raw_x);
      norm_y = jt_normalize_axis(raw_y);

      // Convert to command
      jt_axes_to_cmd(norm_x, norm_y, &linear_x, &angular_z);

      if (linear_x == 0.0f && angular_z == 0.0f)
      {
        // No movement command - small delay to avoid busy loop
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }
    }
    else
    {
      // Disabled -> Don't move
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // Build TwistStamped message using system time from SNTP
    struct timeval tv;
    gettimeofday(&tv, NULL);

    ros_TwistStamped cmd = {};
    cmd.header.stamp.sec     = (int32_t)tv.tv_sec;
    cmd.header.stamp.nanosec = (uint32_t)tv.tv_usec * 1000U;
    // frame_id is an rstring (char*), see my_ros_message_types.h
    cmd.header.frame_id = jt_frame_id_buf;

    cmd.twist.linear.x  = (double)linear_x;
    cmd.twist.linear.y  = 0.0;
    cmd.twist.linear.z  = 0.0;
    cmd.twist.angular.x = 0.0;
    cmd.twist.angular.y = 0.0;
    cmd.twist.angular.z = (double)angular_z;

    size_t len = ps_serialize(jt_pub_buf, &cmd, sizeof(jt_pub_buf));
    if (len > 0)
    {
      picoros_publish(&jt_pub_cmd_vel, jt_pub_buf, len);
    }
    else
    {
      ESP_LOGI(jt_node.name, "TwistStamped message serialization error.");
    }

    // Teleop rate (e.g. 50 Hz)
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// Public entry point; should be called from app_main()
static inline void joystick_teleop()
{
  jt_hw_init();
  jt_ros_init();

  // Publisher task (same pattern as embedded_mobile_base)
  xTaskCreate(jt_publish_cmd_vel_task, "jt_publish_cmd_vel_task", 4096, NULL, 1, NULL);
}

} // namespace joystick_teleop
