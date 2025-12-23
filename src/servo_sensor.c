#include "servo_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "Servo_Task";

bool window_state = false;

// Servo Initialization
esp_err_t servo_init(void)
{
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {SERVO_GPIO},
            .ch = {LEDC_CHANNEL_0},
        },
        .channel_number = 1,
    };

    esp_err_t res = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize servo: %s", esp_err_to_name(res));
        return res;
    }

    ESP_LOGI(TAG, "Servo initialized successfully on GPIO %d", SERVO_GPIO);
    return ESP_OK;
}

// Servo Control Task
void servo_start_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting servo task");
    while (1)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, window_state ? 90 : 0); // open or closed
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}