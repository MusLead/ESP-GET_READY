/* #include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi.h"
#include "mqtt_pub_sub.h"

void app_main(void)
{

    // TO MAKE FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
    ESP_LOGI("MAIN", "[APP] Startup..");
    ESP_LOGI("MAIN", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
    //

    nvs_flash_init();

    // Connect WiFi
    connect_wifi();

    // Start MQTT (connect + subscribe + auto publish logic preserved)
    mqtt_pubsub_start();

    // Publish every 5 seconds
    while (1)
    {
        mqtt_publish("ESP32/values", "Periodic message", 1);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 sek
    }
} */
/*
#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "wifi.h"
#include "mqtt_broker.h"

void app_main(void)
{
    // TO MAKE FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
    ESP_LOGI("MAIN", "[APP] Startup..");
    ESP_LOGI("MAIN", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
    //

    // Always initialize the default NVS partition first
    nvs_flash_init();

    // start WIFI connection
    connect_wifi();

    // Setup AND Start mqtt
    // mqtt_pubsub_start();

    // OR

    //...
    // Set ESP as Broker
    mqtt_broker_start();
} */

/* #include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "wifi.h"
#include "esp_https_server.h"
#include "http_server.h"
#include "mqtt_pub_sub.h"
#include "mqtt_broker.h"

void app_main(void)
{
    // TO MAKE FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
    ESP_LOGI("MAIN", "[APP] Startup..");
    ESP_LOGI("MAIN", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
    //

    // Always initialize the default NVS partition first
    nvs_flash_init();

    // start WIFI connection
    connect_wifi();

    // Start MQTT BROKER --> CORE 0
    xTaskCreatePinnedToCore(mqtt_broker_start, "MQTT BROKER TASK - CORE 0", 4096, NULL, 1, NULL, 0);

    // Start HTTP Server on --> CORE 1
    xTaskCreatePinnedToCore(http_server_start, "HTTP SERVER TASK - CORE 1", 4096, NULL, 1, NULL, 1);
} */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme680_sensor.h"
#include "mqtt_pub_sub.h"
#include "wifi.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    // TO MAKE FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
    ESP_LOGI("MAIN", "[APP] Startup..");
    ESP_LOGI("MAIN", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());
    //

    // Always initialize the default NVS partition first
    nvs_flash_init();

    // start WIFI connection
    connect_wifi();

    // 1 ...
    // INIT THE I2C componennt
    ESP_LOGI(TAG, "Initialisiere I2C Bus...");
    if (i2c_bus_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C Bus Initialisierung fehlgeschlagen!");
        return;
    }
    //
    // ...

    // 2 ...
    // HERE INT ANY OTHER SESNOR FOR UR PROJECT
    // NORMALY u ADD DEVICE SO
    // ESP_ERROR_CHECK(i2c_bus_add_device(&address, &clk_speed, &i2c_master_dev_handle_t));
    // ..
    // BUT - IF library has init use it
    ESP_LOGI(TAG, "Initialisiere BME680 Sensor...");
    if (bme680_sensor_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "BME680 Sensor Initialisierung fehlgeschlagen!");
        return;
    }
    // Servo example:
    // ESP_ERROR_CHECK(servo_init());
    // Optional: Register-Debug
    bme680_print_registers();

    //
    // 3 ...
    // START THE DEVICE

    //
    // 4 ...
    // At the end create a task from the sensor
    ESP_LOGI(TAG, "Starte Lesetask...");
    xTaskCreate(bme680_read_task,
                "bme680_read_task",
                4096,
                NULL,  // Parameter
                3,     // Priorität
                NULL); // Task-Handle

    char msg[32];

    // SECOND TASK
    // ...

    // 5 START UR MQTT SERVICE
    // After SESNOR SETUP
    // Start MQTT (connect + subscribe + auto publish logic preserved)
    mqtt_pubsub_start();

    // egal wo -> Zugriff auf Werte
    while (1)
    {
        snprintf(msg, sizeof(msg),
                 "Temp: %.2f°C,IAQ Score: %u",
                 sensor_data.air_temperature,
                 sensor_data.iaq_score);
        mqtt_publish("ESP32/values", msg, 1);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}