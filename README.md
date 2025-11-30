# ESP32 IoT Project with Espressif

- In lib add the header file for the sensor
- In the src add your sensor .c file
- init in main and start the sesnor before starting the xcreateTask


## Steps FOR USING WIFI

#### Importtant --> alway run nvs_flash_init() before connect_wifi()

After wifi is connected :
```c
- mqtt_pubsub_start() 
```
or 
```c
- mqtt_broker_start() ...
```
---

IF Certificate need for authentication in mqtt_pubsub then add it in new line:

- platform.ini --> board_build.embed_txtfiles
  
**AND**

- src/CMakeLists.txt --> after EMBED_TXTFILES

**AND**

- Pem file in the src/certs folder

## Links for the used libraries
- BME680 : [espressif](https://components.espressif.com/components/k0i05/esp_bme680/versions/1.2.7/readme)
- Servo : [espressif](https://components.espressif.com/components/espressif/servo/versions/0.1.0/readme)
- Mosquitto component : [espressif](https://components.espressif.com/components/espressif/mosquitto/versions/2.0.20~4/readme)

Knowledge:
- I2C : [espressif](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html)
    - I2C with a device : [shawnhymel -- how to use i2c](https://shawnhymel.com/2954/esp32-how-to-use-i2c-with-esp-idf/)
- Http_server: [espressif](https://developer.espressif.com/blog/2025/06/basic_http_server/)
- spiffsgen : [github -- espressif](https://github.com/espressif/esp-idf/blob/master/examples/storage/spiffsgen/main/spiffsgen_example_main.c)
- partition : [github -- espressif](https://github.com/espressif/esp-idf/tree/master/components/partition_table)
- Mqtt :
  - [espressif](https://docs.espressif.com/projects/esp-idf/en/v4.3.3/esp32/api-reference/protocols/mqtt.html)
  - [Medium -- introduction to mqtt](https://medium.com/@vaishalinagori112/introduction-to-mqtt-and-setting-up-a-lab-for-mqtt-pentesting-on-macos-f6018183c2b7)
  - [developer -- espressif](https://developer.espressif.com/blog/2025/05/esp-idf-mosquitto-port/)
