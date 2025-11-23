#pragma once

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include <sys/time.h>

static const char * WIFI_TAG = "WIFI_TIME";

// TODO: set your real SSID/PASSWORD
#define WIFI_SSID      "Your_SSID"
#define WIFI_PASS      "Your_PASSWORD"
#define WIFI_MAX_RETRY 5

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static void wifi_event_handler(
  void * arg,
  esp_event_base_t event_base,
  int32_t event_id,
  void * event_data)
{
  (void)arg;

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (s_retry_num < WIFI_MAX_RETRY)
    {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(WIFI_TAG, "Retrying to connect to the AP");
    }
    else
    {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(WIFI_TAG, "Connect to the AP fail");
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t * event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(
      WIFI_TAG,
      "Got IP: " IPSTR,
      IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_init_sta()
{
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(
    esp_event_handler_instance_register(
      WIFI_EVENT,
      ESP_EVENT_ANY_ID,
      &wifi_event_handler,
      NULL,
      NULL));
  ESP_ERROR_CHECK(
    esp_event_handler_instance_register(
      IP_EVENT,
      IP_EVENT_STA_GOT_IP,
      &wifi_event_handler,
      NULL,
      NULL));

  wifi_config_t wifi_config = {};
  strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
  strcpy((char *)wifi_config.sta.password, WIFI_PASS);
  wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Wait until connected or fail
  EventBits_t bits = xEventGroupWaitBits(
    s_wifi_event_group,
    WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
    pdFALSE,
    pdFALSE,
    portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(WIFI_TAG, "Connected to AP SSID:%s", WIFI_SSID);
  }
  else if (bits & WIFI_FAIL_BIT)
  {
    ESP_LOGE(WIFI_TAG, "Failed to connect to SSID:%s", WIFI_SSID);
  }
  else
  {
    ESP_LOGE(WIFI_TAG, "UNEXPECTED EVENT");
  }
}

static void initialize_sntp()
{
  ESP_LOGI(WIFI_TAG, "Initializing SNTP");
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);

  // Use pool.ntp.org or your local NTP server
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_init();
}

static void obtain_time_via_wifi()
{
  // WiFi and NVS must be initialized before calling this.
  wifi_init_sta();
  initialize_sntp();

  // Wait until time is set
  time_t now = 0;
  struct tm timeinfo = {};
  int retry = 0;
  const int retry_count = 10;

  while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count)
  {
    ESP_LOGI(WIFI_TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    vTaskDelay(pdMS_TO_TICKS(2000));
    time(&now);
    localtime_r(&now, &timeinfo);
  }

  if (timeinfo.tm_year >= (2016 - 1900))
  {
    ESP_LOGI(WIFI_TAG, "System time is set via SNTP");
  }
  else
  {
    ESP_LOGW(WIFI_TAG, "Failed to get time via SNTP");
  }
}
