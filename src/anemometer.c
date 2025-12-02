#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "anemometer.h"

#define SENSOR_PIN 19 // Dein Wind-Sensor-Pin
#define RECORD_TIME 3 // Sekunden

volatile int interrupt_counter = 0;
float wind_speed = 0;

// ISR (Interrupt Service Routine)
static void IRAM_ATTR countup(void *args)
{
    interrupt_counter++;
}

void measure_wind()
{
    interrupt_counter = 0;

    // Interrupt aktivieren
    gpio_isr_handler_add(SENSOR_PIN, countup, (void *)SENSOR_PIN);

    // 3 Sekunden warten
    vTaskDelay(RECORD_TIME * 1000 / portTICK_PERIOD_MS);

    // Interrupt stoppen
    gpio_isr_handler_remove(SENSOR_PIN);

    // Arduino Formel:
    // WindSpeed = pulses / time(sec) * 2.4
    wind_speed = (float)interrupt_counter / (float)RECORD_TIME * 2.4;
}

void anemometer_task(void *pvParameters)
{
    // GPIO konfigurieren
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE};
    gpio_config(&io_conf);

    // ISR Service installieren
    gpio_install_isr_service(0);

    while (1)
    {
        measure_wind();

        printf("Wind Speed: %.2f km/h - %.2f m/s\n",
               wind_speed,
               wind_speed / 3.6);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 Sekunde Pause (optional)
    }
}