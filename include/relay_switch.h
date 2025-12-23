#pragma once
#include <stdbool.h>
#define GPIO_RELAY 18

extern bool fan_state; // false = off, true = on

void relay_start_task(void *pvParameters);