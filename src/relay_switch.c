#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <esp_log.h>

#define GPIO_RELAY 5 // RELAY-PIN (Yello cable)

void relay_blink_task(void *pvParameter)
{
    gpio_reset_pin(GPIO_RELAY);
    gpio_set_direction(GPIO_RELAY, GPIO_MODE_OUTPUT);

    while (true)
    {
        gpio_set_level(GPIO_RELAY, 1);   // Relay on
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1s pause

        gpio_set_level(GPIO_RELAY, 0);   // Relay off
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1s pause
    }
}