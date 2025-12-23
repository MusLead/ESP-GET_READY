// ----
// This entire code is the first and simple implementation to start wifi and start the mqtt_pubsub service and start publishing evry 5 the 'Periodic message' message in the topic 'ESP32/values'
/* #include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi.h"
#include "mqtt_pub_sub.h"

void app_main(void)
{

    // TO FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
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
// ----

//
// ----
// This implemntation starts the wifi and pins two task in two task:
// Core 0 for the mqtt_broker
// Core 1 for the http_server
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
    // TO FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
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
// ----

//
// ----
// This is a combination of mqtt_pubsub with http_server and additionally the bme680 publish every 2 sec the current temp and iaq_score
/* #include "esp_log.h"
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
    // TO FREE UP ANY ALLOCATED HEAP MEMORY AND TO PRINT THE EDF VERSION
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
                 "Temp: %.2f,IAQ Score: %u",
                 sensor_data.air_temperature,
                 sensor_data.iaq_score);
        mqtt_publish("ESP32/values", msg, 1);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
} */
// ----

//
//
// ----
// FINAL IMPLEMENTATION WITH ALL COMPONENTS
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi.h"
#include "mqtt_pub_sub.h"

#include "i2c_bus.h"
#include "bme680_sensor.h"
#include "anemometer.h"

#include "relay_switch.h"
#include "servo_sensor.h"
#include "absorber.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    // 1 System Info
    ESP_LOGI(TAG, "System start");
    ESP_LOGI(TAG, "Free heap: %lu", esp_get_free_heap_size());
    ESP_LOGI(TAG, "IDF version: %s", esp_get_idf_version());

    // 2 Always initialize the default NVS partition first
    nvs_flash_init();

    // 3 WiFi
    connect_wifi();

    // 4 Start MQTT (connect + subscribe + auto publish logic preserved)
    mqtt_pubsub_start();

    //
    // 5 INIT THE I2C componennt
    ESP_ERROR_CHECK(i2c_bus_init());
    //
    // ...

    //
    // 6 SENSORS INIT
    ESP_ERROR_CHECK(bme680_sensor_init());

    // SERVO INIT
    ESP_ERROR_CHECK(servo_init());

    //
    // 7 At the end create a task from the sensor
    // BME680 TASK
    xTaskCreate(bme680_read_task, "bme680_task", 4096, NULL, 3, NULL);
    // ANEMOMETER TASK
    xTaskCreate(anemometer_task, "anemometer_task", 3072, NULL, 4, NULL);
    // RELAY SWITCH TASK
    xTaskCreate(relay_start_task, "relay_task", 2048, NULL, 2, NULL);
    // SERVO TASK
    xTaskCreate(servo_start_task, "servo_task", 2048, NULL, 2, NULL);
    // ABSORBER TASK
    xTaskCreate(absorber_task, "absorber_task", 2048, NULL, 2, NULL);

    ESP_LOGI(TAG, "System initialized successfully");
}
// ----

/* // ----
// reference : https://www.google.com/search?q=i2c_master.h+i2c+scanner+esp+idf&client=safari&hs=ngKU&sca_esv=7059b957e07c44b7&rls=en&biw=1210&bih=724&sxsrf=AE3TifM3WxiSKLym5dsyO_J0y_WvB7Ps5Q%3A1766054242464&ei=YtlDaYONHL6di-gPnNSJ-Qo&ved=0ahUKEwiDxpre-MaRAxW-zgIHHRxqIq8Q4dUDCBE&uact=5&oq=i2c_master.h+i2c+scanner+esp+idf&gs_lp=Egxnd3Mtd2l6LXNlcnAiIGkyY19tYXN0ZXIuaCBpMmMgc2Nhbm5lciBlc3AgaWRmMgUQABjvBTIFEAAY7wUyCBAAGIAEGKIEMggQABiiBBiJBTIFEAAY7wVI4hFQsgFYwA9wAXgBkAEAmAGUAaABnQeqAQM0LjS4AQPIAQD4AQGYAgmgAv4HwgIKEAAYsAMY1gQYR8ICBRAhGKABwgIEECEYFZgDAIgGAZAGCJIHAzMuNqAH-x2yBwMyLja4B_UHwgcFMi04LjHIBzaACAA&sclient=gws-wiz-serp#fpstate=ive&vld=cid:20388e77,vid:Snp6iTu1R7E,st:0
// I2C SCANNER IMPLEMENTATION
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "i2c_bus.h"

static const char *TAG = "MAIN";

void i2c_scanner(void *args)
{
    ESP_LOGI(TAG, "Starting I2C scanner...");

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();

    while (1)
    {
        esp_err_t ret;

        for (int address = 1; address < 127; address++)
        {

            ret = i2c_master_probe(bus, address, 1000);

            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG, "I2C device found at address 0x%X", address);
            }
        }

        printf("I2C scan completed.\n");
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Delay 10 seconds
    }
    vTaskDelete(NULL);
}

void app_main()
{
    // Initialize I2C
    ESP_LOGI(TAG, "Initialisiere I2C Bus...");
    if (i2c_bus_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C Bus Initialisierung fehlgeschlagen!");
        return;
    }

    xTaskCreate(i2c_scanner, "I2C Scanner", 2048, NULL, 2, NULL);
}
 */