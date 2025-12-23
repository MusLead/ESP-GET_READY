#pragma once

// MQTT BROKER ADDRESS
// TODO: Change to your MQTT Broker IP Address
#define MQTT_ADDR_URL "mqtt://10.249.73.216:1883"

void mqtt_pubsub_start(void);
void mqtt_publish(const char *topic, const char *msg, int qos);
void mqtt_subscribe(const char *topic, int qos);