#include "relay_switch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

bool fan_state = false;

void relay_start_task(void *pvParameter)
{
    gpio_reset_pin(GPIO_RELAY);
    gpio_set_direction(GPIO_RELAY, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(GPIO_RELAY, fan_state ? 1 : 0); // Relay on/off
        vTaskDelay(pdMS_TO_TICKS(1000));               // 1s pause
    }
}