# AqiSensor
Air Quality Sensor based on ESP32-WROOM-32 with built-in SSD1306 OLED display and Li-ion battery, complemented with a BME680 sensor to measure temperature, humidity, barometric pressure and volatile organic compounds. It shows the time, date, measured temperature, pressure, humidity and calculated air quality index on the display. It also sends this data every minute via MQTT over WiFi to an home automation platform.
## Usage
At first start-up, the module will act like a WiFi Access Point. Search for the AQIs-sensor SSID and connect your device (e.g. smartphone) to it. It will present you with a configuration menu. It will ask for:
- WiFi SSID
- WiFi password
- MQTT hostname
- MQTT port
- Publish topic to publish the measurements
- Subscribe topic to obtain the time of day

When WiFi nork establishement is successful, these paramwill be stored on the internal flash memory of the ESP32. When the module is rebooted, these parameters will be retrieved. When the WiFi network can be reached, the unit will operate. If not, you need to configure the paramaters again.

## Libraries used in this project:
- Adafruit BME680 Library, version 2.0.2
- Adafruit BusIO, version 1.14.1
- Adafruit GFX Library, version 1.11.5
- Adafruit SSD1306, version 2.5.7
- Adafruit Unified Sensor, version 1.1.9
- Arduinojson, version 6.21.2
- AsyncMQTT_ESP32, version 1.10.0
- AsyncTCP, version 1.1.4, see note 1
- AsyncTCP_SSL, version 1.3.1, see note 1
- ESP8266 and ESP32 OLEDdriver for SSD1306 displays, version 4.4.0
- PubSubClient, version 2.8
- Time, version 1.6.1
- WebServer_ESP32_ENC, version 1.5.3, see note 1
- WebServer_ESP32_SC_ENC, version 1.2.1, see note 1
- WebServer_ESP32_SC_W5500, version 1.2.1, see note 1
- WebServer_ESP32_SC_W6100, version 1.2.1, see note 1
- WebServer_ESP32_W5500, version 1.5.3, see note 1
- WebServer_ESP32_W6100, version 1.5.3, see note 1
- WebServer_ESP32_ETH01, version 1.5.1, see note 1
- WifiManager, version 2.0.15-rc.1

note 1: Installed automatically when installing AsyncMQTT_ESP32 library via the Arduino Library manager.
