#include <stdio.h>
#include "ultrasonic.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h> // ganz oben hinzufügen

void app_main(void)
{
    ultrasonic_sensor_t sensor = {
        .sig_pin = GPIO_NUM_19 // dein SIG-Pin
    };
    ultrasonic_init(&sensor);

    while (1)
    {
        uint32_t distance_cm;
        esp_err_t res = ultrasonic_measure_cm(&sensor, &distance_cm);
        if (res == ESP_OK)
            printf("Entfernung: %" PRIu32 " cm\n", distance_cm);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}