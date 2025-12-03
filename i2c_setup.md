# Modular I2C & Sensor Framework for ESP32

## Project Structure
```
include/
├── i2c_bus.h      # I2C master bus configuration and management
├── bme680_sensor.h # BME680 environmental sensor interface
└── servo_sensor.h  # Servo motor control interface

src/
├── i2c_bus.c      # I2C bus implementation
├── bme680_sensor.c # BME680 sensor implementation
└── servo_sensor.c  # Servo motor implementation
```

## Setup Instructions

### 1. Configure I2C Bus
Before using any I2C devices, configure the I2C pins in `/include/i2c_bus.h`:

```c
// ---- Configurations ----
#define I2C_MASTER_SDA_IO 21    // GPIO pin for SDA
#define I2C_MASTER_SCL_IO 22    // GPIO pin for SCL
#define I2C_MASTER_PORT I2C_NUM_0  // I2C port number
```

**Note:** Adjust these GPIO numbers to match your ESP32 board's physical connections.

### 2. Device-Specific Configurations

#### BME680 Sensor
The BME680 uses I2C address `0x76` by default (can be changed in `bme680_sensor.h`).

#### Servo Motor
Configure the servo in `/include/servo_sensor.h`:
```c
#define SERVO_GPIO 18           // GPIO connected to servo signal wire
#define SERVO_CALIB_0_DEG 30    // Pulse width calibration for 0° position
#define SERVO_CALIB_180_DEG 195 // Pulse width calibration for 180° position
```


## Usage

All components follow this initialization pattern:
1. Initialize the I2C bus (always once when the programm starts)
2. Initialize individual devices
3. Create tasks for continuous operation

### Complete Example
Here's a full `app_main()` example showing how to use all components and u also find it in the main_start_i2c.c file:

```c
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme680_sensor.h"
#include "servo_sensor.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    //
    //
    // Always Init before using any sensor with i2c
    // 1. Initialize I2C Bus (MUST BE FIRST)
    ESP_LOGI(TAG, "Initializing I2C Bus...");
    if (i2c_bus_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C Bus Initialization failed!");
        return;
    }

    //
    //
    // 2. Initialize Sensors
    // Either u want to programm how to read and write data to i2c device then init the sensor using the `i2c_bus_add_device()` function:
    // i2c_bus_add_device(0x48, 100000, &i2c_bus_get_handle());
    // OR
    // u have a init from the library
    
    ESP_LOGI(TAG, "Initializing BME680 Sensor...");
    if (bme680_sensor_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "BME680 Sensor Initialization failed!");
        return;
    }

    ESP_LOGI(TAG, "Initializing Servo...");
    if (servo_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Servo Initialization failed!");
        return;
    }
    
    //
    //
    // 3. Create Tasks for Continuous Operation
    ESP_LOGI(TAG, "Starting sensor tasks...");
    
    // Start BME680 reading task (reads every 10 seconds)
    xTaskCreate(
        bme680_read_task,      // Task function
        "bme680_read_task",    // Task name
        4096,                  // Stack size
        NULL,                  // Parameters
        5,                     // Priority
        NULL                   // Task handle
    );
    //
    // Other Tasks ...
    //
    // Start servo sweeping task
    xTaskCreate(
        servo_start_task,      // Task function
        "Servo Task",          // Task name
        2048,                  // Stack size
        NULL,                  // Parameters
        5,                     // Priority
        NULL                   // Task handle
    );

    // End of the porgramm
}
```

## Component Details

### I2C Bus Manager (`i2c_bus.h/c`)
**Functions:**
- `i2c_bus_init()` - Initialize I2C master bus
- `i2c_bus_get_handle()` - Get bus handle for new i2c device connections
- `i2c_bus_add_device()` - Manually add an I2C device (for custom sensors) ... this can be used if u want to programm the sensor by userself without library

**Manual Device Addition Example:**
```c
esp_err_t ret = i2c_bus_add_device(uint8_t address,
                             uint32_t clk_speed,
                             i2c_master_dev_handle_t *out_dev_handle);
```

### BME680 Environmental Sensor (`bme680_sensor.h/c`)
**Functions:**
- `bme680_sensor_init()` - Initialize sensor
- `bme680_sensor_read()` - Read one measurement
- `bme680_print_registers()` - Optional: print register values [only for debug mode needed]
- `bme680_read_task()` - Continuous reading task

**Data Structure:**
```c
// Global sensor data (updated by read functions)
extern bme680_data_t sensor_data;
// Contains: temperature, humidity, pressure, gas resistance, IAQ score
```

### Servo Motor Control (`servo_sensor.h/c`)
**Functions:**
- `servo_init()` - Initialize servo hardware
- `servo_start_task()` - Task that sweeps servo 0°↔180°

**Manual Servo Control:**
```c
// Move to specific angle (0-180°)
iot_servo_write_angle(LEDC_LOW_SPEED_MODE, 0, 90); // Move to 90°
vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second
```

## Adding New Devices

### Step 1: Create Header File (`include/new_device.h`)
```c
#pragma once
#include "i2c_bus.h"
#include "esp_err.h"

#define NEW_DEVICE_ADDR 0x50

esp_err_t new_device_init(void);
esp_err_t new_device_read_data(void);
void new_device_task(void *pvParameters);
```

### Step 2: Create Implementation (`src/new_device.c`)
```c
#include "new_device.h"
#include "esp_log.h"

static const char *TAG = "NEW_DEVICE";
static i2c_master_dev_handle_t device_handle = NULL;

esp_err_t new_device_init(void)
{
    return i2c_bus_add_device(NEW_DEVICE_ADDR, 100000, &device_handle);
}

// Add your device-specific functions...
```

### Step 3: Use in Main Application
```c
// Init i2c_bus
ESP_LOGI(TAG, "Initializing I2C Bus...");
if (i2c_bus_init() != ESP_OK)
{
    ESP_LOGE(TAG, "I2C Bus Initialization failed!");
    return;
}

// Then Init device
if (new_device_init() != ESP_OK) {
    ESP_LOGE(TAG, "New device initialization failed!");
}


// Create task
xTaskCreate(new_device_task, "New Device", 2048, NULL, 5, NULL);
```
