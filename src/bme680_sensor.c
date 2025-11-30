#include "bme680_sensor.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BME680_SENSOR";

// Der globale Daten-Speicher
bme680_data_t sensor_data = {0};

// BME680 Handle
static bme680_handle_t bme_handle = NULL;

// =======================================
// Init
// =======================================
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

// =======================================
// Eine Messung auslesen
// =======================================
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

// =======================================
// Optional: Register-Debug
// =======================================
void bme680_print_registers(void)
{
    if (!bme_handle)
        return;

    bme680_control_measurement_register_t ctrl_meas;
    bme680_control_humidity_register_t ctrl_humi;
    bme680_config_register_t cfg;
    bme680_control_gas0_register_t gas0;
    bme680_control_gas1_register_t gas1;

    bme680_get_configuration_register(bme_handle, &cfg);
    bme680_get_control_measurement_register(bme_handle, &ctrl_meas);
    bme680_get_control_humidity_register(bme_handle, &ctrl_humi);
    bme680_get_control_gas0_register(bme_handle, &gas0);
    bme680_get_control_gas1_register(bme_handle, &gas1);

    ESP_LOGI(TAG, "===== BME680 Registers =====");
    ESP_LOGI(TAG, "Variant ID     : 0x%02X", bme_handle->variant_id);
    ESP_LOGI(TAG, "Config         : 0x%02X", cfg.reg);
    ESP_LOGI(TAG, "Ctrl Meas      : 0x%02X", ctrl_meas.reg);
    ESP_LOGI(TAG, "Ctrl Hum       : 0x%02X", ctrl_humi.reg);
    ESP_LOGI(TAG, "Ctrl Gas0      : 0x%02X", gas0.reg);
    ESP_LOGI(TAG, "Ctrl Gas1      : 0x%02X", gas1.reg);
    ESP_LOGI(TAG, "============================");
}

// =======================================
// Task-Funktion – wird in main verwendet
// =======================================
void bme680_read_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BME680 read task started");
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        if (bme680_sensor_read() == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "T=%.2f°C H=%.2f%% P=%.2fhPa GAS=%.2fkΩ IAQ=%u (%s)",
                     sensor_data.air_temperature,
                     sensor_data.relative_humidity,
                     sensor_data.barometric_pressure,
                     sensor_data.gas_resistance,
                     sensor_data.iaq_score,
                     bme680_air_quality_to_string(sensor_data.iaq_score));
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