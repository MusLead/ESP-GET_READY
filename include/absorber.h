#pragma once
#include <stdbool.h>
#define GPIO_ABSORBER 26

extern bool absorber_state; // false = off, true = on

void absorber_task(void *pvParameters);