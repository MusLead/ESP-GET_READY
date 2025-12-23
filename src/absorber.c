#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "absorber.h"

bool absorber_state = false;

// ABSORBER TASK
void absorber_task(void *pvParameters)
{
    gpio_reset_pin(GPIO_ABSORBER);
    gpio_set_direction(GPIO_ABSORBER, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(GPIO_ABSORBER, absorber_state ? 1 : 0); // Absorber on/off

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}