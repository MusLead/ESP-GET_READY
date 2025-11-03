#pragma once
#include <driver/gpio.h>
#include <esp_err.h>

typedef struct
{
    gpio_num_t sig_pin; // Single signal pin
} ultrasonic_sensor_t;

esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev);
esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t *time_us);
esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t *distance_cm);
esp_err_t ultrasonic_measure_m(const ultrasonic_sensor_t *dev, float *distance_m);
