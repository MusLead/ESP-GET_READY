#include "mqtt_pub_sub.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "relay_switch.h"
#include "servo_sensor.h"
#include "absorber.h"

static const char *TAG = "MQTT_PUBSUB";

static esp_mqtt_client_handle_t global_client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    global_client = event->client; // Make client accessible in our functions

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Connected to MQTT Broker");
        mqtt_subscribe("ESP32/fan", 1);
        mqtt_subscribe("ESP32/window", 1);
        mqtt_subscribe("ESP32/absorber", 1);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Disconnected from MQTT Broker");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:

        char topic[32];
        char payload[8];

        memcpy(topic, event->topic, event->topic_len);
        topic[event->topic_len] = '\0';

        memcpy(payload, event->data, event->data_len);
        payload[event->data_len] = '\0';

        if (strcmp(topic, "ESP32/fan") == 0)
        {
            fan_state = (payload[0] == '1');
        }
        else if (strcmp(topic, "ESP32/window") == 0)
        {
            window_state = (payload[0] == '1');
        }
        else if (strcmp(topic, "ESP32/absorber") == 0)
        {
            absorber_state = (payload[0] == '1');
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// PUBLIC FUNCTIONS FOR PUBLISH / SUBSCRIBE
void mqtt_publish(const char *topic, const char *msg, int qos)
{
    if (global_client == NULL)
    {
        return;
    }
    esp_mqtt_client_publish(global_client, topic, msg, 0, qos, 1);
}

void mqtt_subscribe(const char *topic, int qos)
{
    if (global_client == NULL)
    {
        return;
    }
    int msg_id = esp_mqtt_client_subscribe(global_client, topic, qos);
    ESP_LOGI(TAG, "Subscribe OK, msg_id=%d", msg_id);
}

// START THE MQTT_PUBSUB
void mqtt_pubsub_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_ADDR_URL,
        .credentials.username = "esp32",
        .credentials.authentication.password = "1234"};

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}