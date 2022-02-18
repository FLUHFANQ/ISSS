#include "WiFi.h"
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include "SHT2x.h"
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;

  // U8G2_SH1107_64X128_F_4W_HW_SPI u8g2(U8G2_R3, /* cs=*/ 14, /* dc=*/ 27, /* reset=*/ 33);

#define WIFI_SSID "Fq"
#define WIFI_PASSWORD "fluhfanq"
#define MQTT_HOST IPAddress(203, 185, 64, 8)   
#define MQTT_PORT 1883
#define MQTT_PUB_WIFI "esp32/test/fang"
#define PIR_Pin 13
#define LDR_Pin 34

unsigned long last = 0;

int LDR;
int PIR;
uint32_t start;
uint32_t stop;
SHT2x sht;
float temperature, humidity;
String ssid;
String rssi;
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
unsigned long previousMillis =100; 
const long interval = 1000000;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  
  switch (event) {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(
        mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("packetId: ");
  Serial.println(packetId);
}

void TCA9548A(uint8_t bus)
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  WiFi.mode(WIFI_STA);
  uint32_t currentFrequency;
  Serial.println("WiFi Netwoek Scan Started");
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer =xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();

  TCA9548A(0);
  sht.begin();
  TCA9548A(1);
  ina219.begin();



//  
//  if (! ina219.begin()) {
//    Serial.println("Failed to find INA219 chip");
//    while (1) { delay(10); }
//  }


//  TCA9548A(0);
//  uint8_t stat = sht.getStatus();
//  Serial.print(stat, HEX);
//  Serial.println();

}

void loop() {
//  Serial.print(": ");
//  Serial.print(WIFI_SSID);
//  Serial.print(": ");
//  Serial.print(WiFi.RSSI());
  
  ssid = WIFI_SSID;
  rssi = WiFi.RSSI();
      
  uint16_t rawTemperature;
  uint16_t rawHumidity;
  
  TCA9548A(0);
  start = micros();
  bool success  = sht.read();
  stop = micros();
  if (success == false)
  {
    Serial.println("SHT20 Not Connect");
  }
  else
  {
    rawTemperature = sht.getRawTemperature();
    rawHumidity = sht.getRawHumidity();
    temperature = sht.getTemperature();  
    humidity = sht.getHumidity();
  }

  
  TCA9548A(1);
  if (!ina219.begin())
  {
    Serial.println("ina219 not connect");
  }
  else
  {
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
  }

  LDR = analogRead(LDR_Pin);
  
  PIR = digitalRead(PIR_Pin);


    if(millis() - last >= 5000)
  {
    String json = "{";
      json += "\"WiFi_SSID\": \""+ssid+"\"";
      json += ", \"WiFi_RSSI\":\""+rssi+"\"";
      json += ", \"Temperature\":"+String(temperature);
      json += ", \"Humidity\":"+String(humidity); 
      json += ", \"Bus Voltage\":"+String(busvoltage);
      json += ", \"Shunt Voltage\":"+String(shuntvoltage);
      json += ", \"Load Voltage\":"+String(loadvoltage);
      json += ", \"Current\":"+String(current_mA);
      json += ", \"Power\":"+String(power_mW);
      json += ", \"LDR\":"+String(LDR);
      json += ", \"PIR\":"+String(PIR);
      json += "}";
  

    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_WIFI, 1, true, json.c_str()); 
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_WIFI);
    Serial.println(packetIdPub1);
   
    Serial.println(json.c_str());
    last = millis();
  }
}
