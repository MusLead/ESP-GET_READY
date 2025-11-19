#include <stdio.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <i2cdev.h>
#include <bme680.h>
#include <string.h>
#include "iot_servo.h"

// BME680 Configuration
#define SDA_GPIO 21
#define SCL_GPIO 22
#define I2C_PORT 0
#define BME680_ADDR BME680_I2C_ADDR_0

// Servo Configuration
#define SERVO_GPIO 18 // Using GPIO 18 for servo to avoid conflict with I2C
static uint16_t calibration_value_0 = 30;
static uint16_t calibration_value_180 = 195;

static const char *TAG = "BME680_Servo";

// Global variables
bme680_t bme680_sensor;

void bme680_init(void)
{
    memset(&bme680_sensor, 0, sizeof(bme680_t));

    ESP_ERROR_CHECK(bme680_init_desc(&bme680_sensor, BME680_ADDR, I2C_PORT, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(bme680_init_sensor(&bme680_sensor));

    // Configure BME680 settings
    bme680_set_oversampling_rates(&bme680_sensor, BME680_OSR_4X, BME680_OSR_2X, BME680_OSR_4X);
    bme680_set_filter_size(&bme680_sensor, BME680_IIR_SIZE_3);
    bme680_set_heater_profile(&bme680_sensor, 0, 300, 150);
    bme680_use_heater_profile(&bme680_sensor, 0);

    ESP_LOGI(TAG, "BME680 initialized successfully!");
}

void servo_init(void)
{
    ESP_LOGI(TAG, "Initializing Servo");

    // Configure the servo
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_GPIO,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };

    // Initialize the servo
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    ESP_LOGI(TAG, "Servo initialized successfully!");
}

void bme680_task(void *pvParameters)
{
    uint32_t duration;
    bme680_get_measurement_duration(&bme680_sensor, &duration);

    bme680_values_float_t values;
    TickType_t last_wakeup = xTaskGetTickCount();

    while (1)
    {
        // Read BME680 sensor data
        if (bme680_force_measurement(&bme680_sensor) == ESP_OK)
        {
            vTaskDelay(duration);

            if (bme680_get_results_float(&bme680_sensor, &values) == ESP_OK)
            {
                printf("BME680: %.2f°C, %.2f%%, %.2fhPa, %.2fΩ\n",
                       values.temperature, values.humidity,
                       values.pressure / 100.0f, values.gas_resistance);

                // Log the data
                ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%, Pressure: %.2fhPa, Gas: %.2fΩ",
                         values.temperature, values.humidity,
                         values.pressure / 100.0f, values.gas_resistance);
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read BME680 results");
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to start BME680 measurement");
        }

        vTaskDelayUntil(&last_wakeup, 2000 / portTICK_PERIOD_MS); // Read every 2 seconds
    }
}

void servo_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Servo Task Started");

    while (1)
    {
        // Move servo based on sensor data or predefined pattern
        ESP_LOGI(TAG, "Moving servo from 0 to 180 degrees");

        // Smooth movement from 0 to 180
        for (int angle = calibration_value_0; angle <= calibration_value_180; angle += 5)
        {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        // Hold at 180 degrees
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Smooth movement back to 0
        for (int angle = calibration_value_180; angle >= calibration_value_0; angle -= 5)
        {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, angle);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

        // Hold at 0 degrees
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "Servo cycle completed, waiting...");
        vTaskDelay(3000 / portTICK_PERIOD_MS); // Wait 3 seconds before next cycle
    }
}

// Advanced: Servo control based on BME680 data
void smart_servo_task(void *pvParameters)
{
    bme680_values_float_t values;
    uint32_t duration;
    bme680_get_measurement_duration(&bme680_sensor, &duration);

    while (1)
    {
        // Read sensor data
        if (bme680_force_measurement(&bme680_sensor) == ESP_OK)
        {
            vTaskDelay(duration);

            if (bme680_get_results_float(&bme680_sensor, &values) == ESP_OK)
            {
                // Control servo based on temperature
                int servo_angle;

                if (values.temperature < 20.0)
                {
                    servo_angle = calibration_value_0; // Cold = 0 degrees
                    ESP_LOGI(TAG, "Cold temperature (%.1f°C), servo at 0°", values.temperature);
                }
                else if (values.temperature < 25.0)
                {
                    servo_angle = (calibration_value_0 + calibration_value_180) / 2; // Medium = 90 degrees
                    ESP_LOGI(TAG, "Medium temperature (%.1f°C), servo at 90°", values.temperature);
                }
                else
                {
                    servo_angle = calibration_value_180; // Hot = 180 degrees
                    ESP_LOGI(TAG, "Hot temperature (%.1f°C), servo at 180°", values.temperature);
                }

                // Move servo to calculated position
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, servo_angle);
            }
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); // Check every 5 seconds
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting BME680 and Servo Application");

    // Initialize I2C
    ESP_ERROR_CHECK(i2cdev_init());

    // Initialize hardware
    bme680_init();
    servo_init();

    // Create tasks
    xTaskCreate(bme680_task, "bme680_task", 4096, NULL, 5, NULL);

    // Choose one of the servo tasks:

    // Option 1: Simple servo movement (independent)
    // xTaskCreate(servo_task, "servo_task", 2048, NULL, 4, NULL);

    // Option 2: Smart servo controlled by BME680 data (comment above line and uncomment below)
    xTaskCreate(smart_servo_task, "smart_servo_task", 4096, NULL, 4, NULL);

    ESP_LOGI(TAG, "All tasks created successfully!");

    // Keep the main task alive
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*

// -- SERVO --

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "iot_servo.h"

#define SERVO_GPIO (22) // Servo GPIO

static const char *TAG = "Servo Control";

static uint16_t calibration_value_0 = 30;    // Real 0 degree angle
static uint16_t calibration_value_180 = 195; // Real 0 degree angle

// Task to test the servo
static void servo_test_task(void *arg)
{
    ESP_LOGI(TAG, "Servo Test Task");
    while (1)
    {
        // Set the angle of the servo
        for (int i = calibration_value_0; i <= calibration_value_180; i += 1)
        {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, i);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        // Return to the initial position
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, calibration_value_0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void servo_init(void)
{
    ESP_LOGI(TAG, "Servo Control");

    // Configure the servo
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = {
            .servo_pin = {
                SERVO_GPIO,
            },
            .ch = {
                LEDC_CHANNEL_0,
            },
        },
        .channel_number = 1,
    };

    // Initialize the servo
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Servo Control");
    // Initialize the servo
    servo_init();
    // Create the servo test task
    xTaskCreate(servo_test_task, "servo_test_task", 2048, NULL, 5, NULL);
} */

/* // --- BME 680 ---
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bme680.h>
#include <i2cdev.h>
#include <string.h>

// --- Sensor 1 ---
#define SDA1 21
#define SCL1 22
#define PORT1 I2C_NUM_0
#define ADDR1 BME680_I2C_ADDR_0 // 0x76

// --- Sensor 2 ---
#define SDA2 19
#define SCL2 18
#define PORT2 I2C_NUM_1
#define ADDR2 BME680_I2C_ADDR_0 // gleiche Adresse, anderer Bus

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

void bme680_task(void *pvParameters)
{
    // Configuration Struct
    struct
    {
        int port;
        int sda;
        int scl;
        uint8_t addr;
        const char *name;
    } *cfg = pvParameters;

    bme680_t sensor;
    memset(&sensor, 0, sizeof(sensor));

    ESP_ERROR_CHECK(bme680_init_desc(&sensor, cfg->addr, cfg->port, cfg->sda, cfg->scl));
    ESP_ERROR_CHECK(bme680_init_sensor(&sensor));

    bme680_set_oversampling_rates(&sensor, BME680_OSR_4X, BME680_OSR_NONE, BME680_OSR_2X);
    bme680_set_filter_size(&sensor, BME680_IIR_SIZE_7);
    bme680_set_heater_profile(&sensor, 0, 200, 100);
    bme680_use_heater_profile(&sensor, 0);
    bme680_set_ambient_temperature(&sensor, 10);

    uint32_t duration;
    bme680_get_measurement_duration(&sensor, &duration);

    TickType_t last_wakeup = xTaskGetTickCount();
    bme680_values_float_t values;

    while (1)
    {
        if (bme680_force_measurement(&sensor) == ESP_OK)
        {
            vTaskDelay(duration);

            if (bme680_get_results_float(&sensor, &values) == ESP_OK)
            {
                printf("%s: %.2f °C, %.2f %%RH, %.2f hPa, %.2f Ohm\n",
                       cfg->name,
                       values.temperature,
                       values.humidity,
                       values.pressure,
                       values.gas_resistance);
            }
        }
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    static const struct
    {
        int port;
        int sda;
        int scl;
        uint8_t addr;
        const char *name;
    } sensors[] = {
        {PORT1, SDA1, SCL1, ADDR1, "BME680_1"},
        {PORT2, SDA2, SCL2, ADDR2, "BME680_2"}};

    xTaskCreatePinnedToCore(bme680_task, "bme680_1", 4096, (void *)&sensors[0], 5, NULL, APP_CPU_NUM);
    xTaskCreatePinnedToCore(bme680_task, "bme680_2", 4096, (void *)&sensors[1], 5, NULL, APP_CPU_NUM);
}
*/

/*
// ---- OLED DISPLAY -----
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ssd1306.h>

#define I2C_SDA 21 // SERIAL DATA LINE
#define I2C_SCL 22 // SERIAL CLOCK LINE

void app_main(void)
{
    // Display initialisieren (Standard-Adresse 0x3C)
    ssd1306_128x64_i2c_initEx(I2C_SCL, I2C_SDA, 0);
    ssd1306_clearScreen();
    ssd1306_setFixedFont(ssd1306xled_font8x16);

    // Bildschirm löschen
    ssd1306_clearScreen();

    // Text ausgeben
    ssd1306_printFixed(0, 10, "Hello World!", STYLE_NORMAL);
} */