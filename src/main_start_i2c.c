//
// Here is the tested code which start i2c and starts the bme680 and servo at the same time
/* #include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "bme680_sensor.h"
#include "servo_sensor.h"

static const char *TAG = "MAIN";

void app_main(void)
{
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
    ESP_ERROR_CHECK(servo_init());

    //
    // 3 ...
    // START THE DEVICE

    bme680_sensor_start_measurement();

    //
    // 4 ...
    // At the end create a task from the sensor
    ESP_LOGI(TAG, "Starte Lesetask...");
    xTaskCreate(bme680_read_task,
                "bme680_read_task",
                4096,
                NULL,  // Parameter
                5,     // Priorität
                NULL); // Task-Handle
    // SECOND TASK
    xTaskCreate(servo_start_task, "Servo Task", 2048, NULL, 5, NULL);
} */