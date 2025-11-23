#include <math.h>  // For fmod() and M_PI
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "led_strip_esp32.h"
#include "nvs_flash.h"
#include "flashing_leds.h"
#include "embedded_mobile_base.h"

extern "C" void app_main(void)
{
  embedded_mobile_base();
  // flashing_leds();
}
