#include <esp_idf_lib_helpers.h>
#include "ultrasonic.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_err.h>

#define TRIGGER_LOW_DELAY_US 2
#define TRIGGER_HIGH_DELAY_US 5  // wie Arduino
#define ECHO_TIMEOUT_US 1000000  // 1s Timeout (wie Grove)
#define SOUND_SPEED_DIV_CM 58.0f // us per cm

esp_err_t ultrasonic_init(const ultrasonic_sensor_t *dev)
{
    if (!dev)
        return ESP_ERR_INVALID_ARG;
    gpio_set_direction(dev->sig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->sig_pin, 0);
    return ESP_OK;
}

esp_err_t ultrasonic_measure_raw(const ultrasonic_sensor_t *dev, uint32_t *time_us)
{
    if (!dev || !time_us)
        return ESP_ERR_INVALID_ARG;

    // Trigger-Puls
    gpio_set_direction(dev->sig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->sig_pin, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY_US);
    gpio_set_level(dev->sig_pin, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY_US);
    gpio_set_level(dev->sig_pin, 0);

    // Wechsel auf Eingang
    gpio_set_direction(dev->sig_pin, GPIO_MODE_INPUT);

    int64_t start = esp_timer_get_time();
    // Warten auf steigende Flanke
    while (gpio_get_level(dev->sig_pin) == 0)
    {
        if ((esp_timer_get_time() - start) > ECHO_TIMEOUT_US)
            return ESP_ERR_TIMEOUT;
    }
    int64_t echo_start = esp_timer_get_time();

    // Warten auf fallende Flanke
    while (gpio_get_level(dev->sig_pin) == 1)
    {
        if ((esp_timer_get_time() - echo_start) > ECHO_TIMEOUT_US)
            return ESP_ERR_TIMEOUT;
    }
    int64_t echo_end = esp_timer_get_time();

    *time_us = (uint32_t)(echo_end - echo_start);

    // Pin wieder auf LOW setzen (wie Arduino)
    gpio_set_direction(dev->sig_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->sig_pin, 0);

    return ESP_OK;
}

esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t *distance_cm)
{
    if (!dev || !distance_cm)
        return ESP_ERR_INVALID_ARG;
    uint32_t echo_time;
    esp_err_t res = ultrasonic_measure_raw(dev, &echo_time);
    if (res != ESP_OK)
        return res;

    *distance_cm = (uint32_t)((float)echo_time / SOUND_SPEED_DIV_CM + 0.5f);
    return ESP_OK;
}

esp_err_t ultrasonic_measure_m(const ultrasonic_sensor_t *dev, float *distance_m)
{
    if (!dev || !distance_m)
        return ESP_ERR_INVALID_ARG;
    uint32_t echo_time;
    esp_err_t res = ultrasonic_measure_raw(dev, &echo_time);
    if (res != ESP_OK)
        return res;

    *distance_m = ((float)echo_time / SOUND_SPEED_DIV_CM) / 100.0f;
    return ESP_OK;
}
