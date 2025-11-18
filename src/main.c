/* --- BME 680 --- */
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