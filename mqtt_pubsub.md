# Modular I2C & Network Framework for ESP32

## Project Structure
```
include/
├── i2c_bus.h         # I2C master bus configuration
├── bme680_sensor.h   # BME680 environmental sensor
├── servo_sensor.h    # Servo motor control
├── mqtt_broker.h     # MQTT broker server
├── http_server.h     # HTTP web server
├── wifi.h           # WiFi connection
└── mqtt_pub_sub.h   # MQTT client functions

src/
├── i2c_bus.c
├── bme680_sensor.c
├── servo_sensor.c
├── mqtt_broker.c
├── http_server.c
└── wifi.c, mqtt_pub_sub.c
```

## Quick Setup

### 1. Configure Hardware
```c
// in include/i2c_bus.h
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22

// in the bme680_sensor.h
#define BME680_I2C_ADDRESS 0x76

// in include/servo_sensor.h
#define SERVO_GPIO 18
```

### 2. Copy Main Application
```c
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
    // At the end create a task from the sensor
    ESP_LOGI(TAG, "Starte Lesetask...");
    xTaskCreate(bme680_read_task,
                "bme680_read_task",
                4096,
                NULL,  // Parameter
                3,     // Priorität
                NULL); // Task-Handle

    // SECOND TASK
    // ...

    // 5 START UR MQTT PUBSUB SERVICE
    // After SESNOR SETUP
    // Start MQTT PUBSUB (connect + auto publish every 2 sec)
    mqtt_pubsub_start();

    char msg[32]; // message that needs to be send to MQTT broker

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
```

## Infos for client user

**Requirements**: Place these files in `main/certs/`:
- `client.crt` - Server certificate
- `client.key` - Server private key
- `ca.crt` - CA certificate

### MQTT Client (`mqtt_pub_sub.h`)
**Purpose**: Connects to external MQTT brokers and publishes sensor data

**Usage**:
```c
// Start MQTT client
mqtt_pubsub_start();

// Publish data
mqtt_publish("topic/name", "message", qos);

// Subscribe data
mqtt_subscribe("topic/name", qos)
```

**QOS options**

| QoS Level | Delivery Guarantee | Duplicates | Network Usage
|-----------|-------------------|------------|---------------|
| 0 | No guarantee | No | Lowest |
| 1 | Guaranteed | Possible | Medium |
| 2 | Guaranteed | No | Highest |


## Access Your Data

### Option 1: HTTP (Easiest)
```
# In browser or with curl:
curl http://192.168.1.100/
# Response: { "temp": "23.5" }
```

### Option 2: MQTT
```
# Connect to ESP32's broker (port 8883)
# Subscribe to: ESP32/values
```
