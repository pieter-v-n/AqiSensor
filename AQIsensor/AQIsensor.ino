#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <AsyncMQTT_ESP32.h>      
#include <FS.h>                   // File System Library
#include <SPIFFS.h>               // SPI Flash Syetem Library
#include <ArduinoJson.h>          // Arduino JSON library
#include <WiFi.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <TimeLib.h>              // Install https://github.com/PaulStoffregen/Time

// Include the correct display library
// For a connection via I2C using Wire include
#include <Wire.h>         // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"  // legacy include: `#include "SSD1306.h"`

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define _ASYNC_MQTT_LOGLEVEL_ 1
// JSON configuration file
#define JSON_CONFIG_FILE "/aqi_config.json"

#define SDA 5
#define SCL 4
#define LED 16

#define CONNECT_ON 5000    // connectivity on for 5 seconds
#define CONNECT_OFF 50000  // connectivy off for 50 seconds
#define MEASURE_INTERVAL 60000    // measure every 60 seconds

#define TZ (2 * 3600)  // add 2 hours to UTC time

#define SEALEVELPRESSURE_HPA (1013.25)

// Include the UI lib
#include "OLEDDisplayUi.h"

// Include custom images
#include "images.h"

// Defauklts for Variables to hold data from custom textboxes
char MqttHost[50] = "thermis.nl";       // Host to connect to for MQTTY
int  MqttPort = 1883;                   // Port to connect to for MQTT
char PubTopic[50] = "esp32/aqisensor";  // Topic to publish
char SubTopic[50] = "server/time";      // Topic to subscribe

// Flag for saving data

bool shouldSaveConfig = false;

WiFiManager wifiManager;
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
TimerHandle_t bme680MeasureTimer;

int WiFiConnected = 0;

Adafruit_BME680 bme;  // I2C
int aqi = 0;

// Initialize the OLED display using Wire library
SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
OLEDDisplayUi ui(&display);

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW / 2;
int clockCenterY = ((screenH - 16) / 2) + 16;  // top yellow part is 16 px height
int clockRadius = 23;

// utility function for digital clock display: prints leading 0
String twoDigits(int digits) {
  if (digits < 10) {
    String i = '0' + String(digits);
    return i;
  } else {
    return String(digits);
  }
}

void clockOverlay(OLEDDisplay* display, OLEDDisplayUiState* state) {
}

void analogClockFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  //  ui.disableIndicator();

  // Draw the clock face
  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
  display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
  //
  //hour ticks
  for (int z = 0; z < 360; z = z + 30) {
    //Begin at 0° and stop at 360°
    float angle = z;
    angle = (angle / 57.29577951);  //Convert degrees to radians
    int x2 = (clockCenterX + (sin(angle) * clockRadius));
    int y2 = (clockCenterY - (cos(angle) * clockRadius));
    int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 8))));
    int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 8))));
    display->drawLine(x2 + x, y2 + y, x3 + x, y3 + y);
  }

  // display second hand
  /*
  float angle = second() * 6;
  angle = (angle / 57.29577951);  //Convert degrees to radians
  int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 5))));
  int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 5))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
  //
  */

  // display minute hand
  float angle = minute() * 6;
  angle = (angle / 57.29577951);  //Convert degrees to radians
  int x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 4))));
  int y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 4))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
  //
  // display hour hand
  angle = hour() * 30 + int((minute() / 12) * 6);
  angle = (angle / 57.29577951);  //Convert degrees to radians
  x3 = (clockCenterX + (sin(angle) * (clockRadius - (clockRadius / 2))));
  y3 = (clockCenterY - (cos(angle) * (clockRadius - (clockRadius / 2))));
  display->drawLine(clockCenterX + x, clockCenterY + y, x3 + x, y3 + y);
}

void digitalClockFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, timenow);
}

void digitalDateFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String daynow = String(day()) + "-" + twoDigits(month()) + "-" + twoDigits(year());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, daynow);
}

void textFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_16);
  display->drawString(clockCenterX + x, screenH - 32 + y, "Luchtkwaliteit");
  if (aqi <= 50) display->drawString(clockCenterX + x, screenH - 16, "Goed");
  else if (aqi <= 100) display->drawString(clockCenterX + x, screenH - 16 + y, "Redelijk");
  else if (aqi <= 150) display->drawString(clockCenterX + x, screenH - 16 + y, "Onvoldoende");
  else if (aqi <= 200) display->drawString(clockCenterX + x, screenH - 16 + y, "Slecht");
  else if (aqi <= 300) display->drawString(clockCenterX + x, screenH - 16 + y, "Heel slecht");
  else display->drawString(clockCenterX + x, screenH - 16 + y, "Gevaarlijk");
}

void tempFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, String(bme.temperature,2) + " C");
}

void humFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, String(bme.humidity,2) + " %");
}

void pressFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, String((bme.pressure/100.0),2) + " hPa");
}

void aqiFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x, clockCenterY + y, String(aqi) + " AQI");
}

void wifiFrame(OLEDDisplay* display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_16);
  display->drawString(clockCenterX + x, screenH - 16 + y, WiFi.localIP().toString());
  if (WiFiConnected == 1) {
    display->drawString(clockCenterX + x, screenH - 32 + y, "Verbonden");
  } else {
    display->drawString(clockCenterX + x, screenH - 32 + y, "Verbroken");
  };
}
// This array keeps function pointers to all frames; frames are the single views that slide in
FrameCallback frames[] = { analogClockFrame, digitalClockFrame, digitalDateFrame, textFrame, wifiFrame, tempFrame, humFrame, pressFrame, aqiFrame };

// how many frames are there?
int frameCount = 9;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { clockOverlay };
int overlaysCount = 1;


void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  if (wifiManager.autoConnect("AQI-sensor")) {
    Serial.println("Connected to Wi-Fi");
  } else {
    Serial.println("Failed Connecting to Wi-Fi");
    ESP.restart();
  };
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.setKeepAlive(120);  // avoid too many MQTT ping requests
  mqttClient.connect();
}

void bme680Measure(TimerHandle_t xTimer) {
  Serial.println("Start BME680 measurement....");
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(F(" KOhms"));

  Serial.print(F("Approx. Altitude = "));
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(F(" m"));

  // Calculate aqi
  float qt, qh, qg;
  qt = (bme.temperature > 21) ? (bme.temperature - 21) / 79 : (21 - bme.temperature) / 41;
  qh = (bme.humidity > 40) ? (bme.humidity - 40) / 60 : (40 - bme.humidity) / 40;
  qg = (bme.gas_resistance > 50000) ? 0 : (50000 - bme.gas_resistance) / 50000;
  aqi = 50 * qt + 50 * qh + 400 * qg;

 
  Serial.println(qt);
  Serial.println(qh);
  Serial.println(qg);
  Serial.print(F("AQI = "));
  Serial.println(aqi);

  Serial.println();
    
  String PubPayload = "{";
  PubPayload.concat("\"timestamp\": ");PubPayload.concat(now()-TZ);PubPayload.concat("000");
  PubPayload.concat(",\"temperature\": ");PubPayload.concat(bme.temperature);
  PubPayload.concat(",\"humidity\": ");PubPayload.concat(bme.humidity);
  PubPayload.concat(",\"pressure\": ");PubPayload.concat(bme.pressure/100.0);
  PubPayload.concat(",\"resistance\": ");PubPayload.concat(bme.gas_resistance);
  PubPayload.concat(",\"co2\": ");PubPayload.concat(0.0);
  PubPayload.concat(",\"aqi\": ");PubPayload.concat(aqi);
  PubPayload.concat("}");
  if (bme.pressure > 0) {
    mqttClient.publish(PubTopic, 0, true, PubPayload.c_str());
    Serial.println("Publishing at QoS 0");
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
#if USING_CORE_ESP32_CORE_V200_PLUS
      // true
    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi core ready");
      WiFiConnected = 0;
      digitalWrite(LED, LOW);
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi STA core starting");
      WiFiConnected = 0;
      digitalWrite(LED, LOW);
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi STA core connected");
      WiFiConnected = 1;
      digitalWrite(LED, HIGH);
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi core connected");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      WiFiConnected = 1;
      digitalWrite(LED, HIGH);
      //bme680Measure();
      connectToMqtt();
      
      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("WiFi core lost IP");
      WiFiConnected = 0;
      digitalWrite(LED, LOW);
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi core lost connection");
      xTimerStop(mqttReconnectTimer, 0);                            // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, pdMS_TO_TICKS(CONNECT_OFF));  // wait some time before reconnecting again
      WiFiConnected = 0;
      digitalWrite(LED, LOW);
      break;
#else

    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi system connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      WiFiConnected = 1;
      digitalWrite(LED, HIGH);
      connectToMqtt();
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi system lost connection");
      xTimerStop(mqttReconnectTimer, 0);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      WiFiConnected = 0;
      digitalWrite(LED, LOW);
      break;
#endif

    default:
      break;
  }
}

void onMqttConnect(bool sessionPresent) {

  String PubPayload = "{";
  PubPayload.concat("\"timestamp\": ");PubPayload.concat(now()-TZ);PubPayload.concat("000");
  PubPayload.concat(",\"temperature\": ");PubPayload.concat(bme.temperature);
  PubPayload.concat(",\"humidity\": ");PubPayload.concat(bme.humidity);
  PubPayload.concat(",\"pressure\": ");PubPayload.concat(bme.pressure/100.0);
  PubPayload.concat(",\"resistance\": ");PubPayload.concat(bme.gas_resistance);
  PubPayload.concat(",\"co2\": ");PubPayload.concat(0.0);
  PubPayload.concat(",\"aqi\": ");PubPayload.concat(aqi);
  PubPayload.concat("}");

  Serial.print("Connected to MQTT broker: ");
  Serial.print(MqttHost);
  Serial.print(", port: ");
  Serial.println(MqttPort);
  Serial.print("PubTopic: ");
  Serial.println(PubTopic);
  Serial.println(PubPayload);

  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(SubTopic, 1);
  Serial.print("Subscribing at QoS 1, packetId: ");
  Serial.println(packetIdSub);

  if (bme.pressure > 0) {
    mqttClient.publish(PubTopic, 0, true, PubPayload.c_str());
    Serial.println("Publishing at QoS 0");
  }
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void)reason;

  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(const uint16_t& packetId, const uint8_t& qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(const uint16_t& packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, const AsyncMqttClientMessageProperties& properties,
                   const size_t& len, const size_t& index, const size_t& total) {
  (void)payload;
/*
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  Serial.print("  payload: ");
  Serial.println(payload);
  */
  setTime((atoll(payload) / 1000) + TZ);  // received in UTC, need to adjust for timezone
}

void onMqttPublish(const uint16_t& packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void saveConfigFile()
// Save Config in JSON format
{
  Serial.println(F("Saving configuration..."));
  
  // Create a JSON document
  StaticJsonDocument<512> json;
  json["MqttHost"] = MqttHost;
  json["MqttPort"] = MqttPort;
  json["PubTopic"] = PubTopic;
  json["SubTopic"] = SubTopic;

  // Open config file
  File configFile = SPIFFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile)
  {
    // Error, file did not open
    Serial.println("failed to open config file for writing");
  }
 
  // Serialize JSON data to write to file
  serializeJsonPretty(json, Serial);
  if (serializeJson(json, configFile) == 0)
  {
    // Error writing file
    Serial.println(F("Failed to write to file"));
  }
  // Close file
  configFile.close();
}
 
bool loadConfigFile()
// Load existing configuration file
{
  // Uncomment if we need to format filesystem
  // SPIFFS.format();
 
  // Read configuration from FS json
  Serial.println("Mounting File System...");
 
  // May need to make it begin(true) first time you are using SPIFFS
  if (SPIFFS.begin(false) || SPIFFS.begin(true))
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists(JSON_CONFIG_FILE))
    {
      // The file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open(JSON_CONFIG_FILE, "r");
      if (configFile)
      {
        Serial.println("Opened configuration file");
        StaticJsonDocument<512> json;
        DeserializationError error = deserializeJson(json, configFile);
        serializeJsonPretty(json, Serial);
        if (!error)
        {
          Serial.println("Parsing JSON");
 
          strcpy(MqttHost, json["MqttHost"]);
          MqttPort = json["MqttPort"].as<int>();
          strcpy(PubTopic, json["PubTopic"]);
          strcpy(SubTopic, json["SubTopic"]);
          return true;
        }
        else
        {
          // Error loading JSON data
          Serial.println("Failed to load json config");
        }
      }
    }
  }
  else
  {
    // Error mounting file system
    Serial.println("Failed to mount FS");
  }
 
  return false;
}
 
void saveConfigCallback()
// Callback notifying us of the need to save configuration
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
 
void configModeCallback(WiFiManager *myWiFiManager)
// Called when config mode launched
{
  Serial.println("Entered Configuration Mode");
 
  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
 
  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000)
    ;
  delay(500);

  // Remove any previous network settings
  // wifiManager.resetSettings();

  pinMode(LED, OUTPUT);

    // Change to true when testing to force configuration every time we run
  bool forceConfig = false;
 
  bool spiffsSetup = loadConfigFile();
  if (!spiffsSetup)
  {
    Serial.println(F("Forcing config mode as there is no saved config"));
    forceConfig = true;
  }

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // it is a good practice to make sure your code sets wifi mode how you want it.
  
  // Reset settings (only for development)
  // wifiManager.resetSettings();

  // Set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  // Set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // Custom elements

  // Text box (String) - 50 characters maximum
  WiFiManagerParameter mqtthost("key_text", "Enter MQTT Hostname", MqttHost, 50);
  
  // Need to convert numerical input to string to display the default value.
  char convertedValue[6];
  sprintf(convertedValue, "%d", MqttPort); 
    // Text box (Number) - 7 characters maximum
  WiFiManagerParameter mqttport("key_num", "Enter MQTT Port number", convertedValue, 7); 

  // Text box (String) - 50 characters maximum
  WiFiManagerParameter pubtopic("key_text", "Enter Publish Topic", PubTopic, 50);

    // Text box (String) - 50 characters maximum
  WiFiManagerParameter subtopic("key_text", "Enter Subscribe Topic", SubTopic, 50);

  // Add all defined parameters
  wifiManager.addParameter(&mqtthost);
  wifiManager.addParameter(&mqttport);
  wifiManager.addParameter(&pubtopic);
  wifiManager.addParameter(&subtopic);


  Wire.setPins(SDA, SCL);

  Serial.println("Starting up...");
  Serial.println(ARDUINO_BOARD);
  Serial.println(ASYNC_MQTT_ESP32_VERSION);

  Serial.println(F("BME680 async test"));
  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1)
      ;
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms

  bme680MeasureTimer = xTimerCreate("bme680Timer", pdMS_TO_TICKS(MEASURE_INTERVAL), pdTRUE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(bme680Measure));
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(CONNECT_ON), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(CONNECT_ON), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

//  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setServer(MqttHost, MqttPort);

  // We start by connecting to a WiFi network
  connectToWifi();

  xTimerStart(bme680MeasureTimer, 0);

  // The ESP is capable of rendering 60fps in 80Mhz mode
  // but that won't give you much time for anything else
  // run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(60);

  // Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  // display.flipScreenVertically();
}

void loop() {
  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.

    delay(remainingTimeBudget);
  }
}
