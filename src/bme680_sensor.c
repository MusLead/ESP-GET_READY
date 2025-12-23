#include "bme680_sensor.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mqtt_pub_sub.h"

static const char *TAG = "BME680_SENSOR";

// BME680 Sensor Data
bme680_data_t sensor_data = {0};

// BME680 Handle
static bme680_handle_t bme_handle = NULL;

// INIT THE BME680 SENSOR
esp_err_t bme680_sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing BME680...");

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (!bus)
    {
        ESP_LOGE(TAG, "I2C bus NOT ready!");
        return ESP_FAIL;
    }

    bme680_config_t cfg = I2C_BME680_CONFIG_DEFAULT;
    cfg.i2c_address = BME680_I2C_ADDRESS;

    esp_err_t res = bme680_init(bus, &cfg, &bme_handle);
    if (res != ESP_OK || !bme_handle)
    {
        ESP_LOGE(TAG, "bme680_init failed: %s", esp_err_to_name(res));
        return res;
    }

    ESP_LOGI(TAG, "BME680 initialized OK");
    return ESP_OK;
}

// READ SENSOR DATA
esp_err_t bme680_sensor_read(void)
{
    if (!bme_handle)
    {
        ESP_LOGE(TAG, "Sensor not initialized!");
        return ESP_FAIL;
    }

    esp_err_t ret = bme680_get_data(bme_handle, &sensor_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "bme680_get_data failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Druck & Gas-Werte umrechnen
    sensor_data.barometric_pressure /= 100.0f; // hPa
    sensor_data.gas_resistance /= 1000.0f;     // kΩ

    return ESP_OK;
}

// BME680 READ TASK
void bme680_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BME680 read task started");
    TickType_t last_wake_time = xTaskGetTickCount();
    char msg[64];

    while (1)
    {
        if (bme680_sensor_read() == ESP_OK)
        {
            snprintf(msg, sizeof(msg),
                     "Temp: %.2f, H=%.2f, AQ: %u",
                     sensor_data.air_temperature,
                     sensor_data.relative_humidity,
                     sensor_data.iaq_score);

            mqtt_publish("ESP32/indoor", msg, 1); // change this to "ESP32/outdoor" later
        }

        vTaskDelayUntil(&last_wake_time, 10); // macht eine absolute Verzögerung --> Zyklen bleiben stabil und präzise.

        // Unterschied zu vTaskDelay(10000 / portTICK_PERIOD_MS)
        // vTaskDelay() macht eine relative Verzögerung:
        // War die Task zu langsam, verschiebt sich der Zyklus.
    }

    //
    // free resources
    bme680_delete(bme_handle);
    vTaskDelete(NULL);
}