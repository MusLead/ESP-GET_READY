#pragma once
#include "bme680.h"
#include "esp_err.h"

#define BME680_I2C_ADDRESS 0x76
extern bme680_data_t sensor_data;

esp_err_t bme680_sensor_init(void);
esp_err_t bme680_sensor_read(void);
void bme680_read_task(void *pvParameters);
void bme680_print_registers(void);
