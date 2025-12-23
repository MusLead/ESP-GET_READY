#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "anemometer.h"
#include "mqtt_pub_sub.h"

volatile int interrupt_counter = 0;
float wind_speed = 0;

static void IRAM_ATTR countup(void *args)
{
    interrupt_counter++;
}

void measure_wind()
{

    interrupt_counter = 0; // start interrupt

    gpio_isr_handler_add(SENSOR_PIN, countup, (void *)SENSOR_PIN); // wait for RECORD_TIME seconds

    vTaskDelay(RECORD_TIME * 1000 / portTICK_PERIOD_MS); // stop interrupt

    gpio_isr_handler_remove(SENSOR_PIN);

    // Arduino Formel:
    // WindSpeed = pulses / time(sec) * 2.4
    wind_speed = (float)interrupt_counter / (float)RECORD_TIME * 2.4;
}

void anemometer_task(void *pvParameters)
{
    // GPIO config for input pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };

    gpio_config(&io_conf); // install ISR service

    gpio_install_isr_service(0);

    char msg[32];

    while (1)
    {
        measure_wind();
        snprintf(msg, sizeof(msg), "%.2f", wind_speed);
        mqtt_publish("ESP32/wind", msg, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // wait 1 second before next measurement
    }
}