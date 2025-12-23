#pragma once

#define SENSOR_PIN 19 // GPIO pin connected to the anemometer
#define RECORD_TIME 3 // Measurement duration in seconds

void anemometer_task(void *pvParameters);