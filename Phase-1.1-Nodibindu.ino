// GPS, IMU, TEMPERATURE, TURBIDITY

// Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#include <Adafruit_BNO055.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// Pin & Object Initialization
#define RXD1 12
#define TXD1 13
#define MODEM_BAUDRATE 115200
HardwareSerial SerialAT(1);

// DHT22 Sensor
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// BNO055 IMU Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// DS18B20 A & B Temperature Sensors
#define DS18B20A_PIN 7
OneWire oneWireA(DS18B20A_PIN);
DallasTemperature ds18b20a(&oneWireA);
#define DS18B20B_PIN 8
OneWire oneWireB(DS18B20B_PIN);
DallasTemperature ds18b20b(&oneWireB);
#define LED_PIN 2

//Turbidity Sensor
#define TURBIDITY_ANALOG_PIN 7

// MQTT Configuration
const char mqttServer[] = "io.adafruit.com";
const int mqttPort = 1883;
const char mqttUser[] = "uSER-NAME";
const char mqttPassword[] = "AIO-pin";
const char mqttPublishTopic[] = "USER-NAME/feeds/FEED-NAME";
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 5000;

// Function to Send AT Commands with Timeout
String sendATCommand(const String &command, unsigned long timeout) {
  String response = "";
  while (SerialAT.available()) SerialAT.read(); // Clear buffer
  SerialAT.println(command);
  delay(timeout);
  while (SerialAT.available()) response += (char)SerialAT.read();
  response.trim();
  return response;
}

// Function to Extract Values from Data
String getValue(const String &data, char separator, int index) {
  int found = 0;
  int start = 0;
  int end = -1;
  for (int i = 0; i < data.length(); i++) {
    if (data.charAt(i) == separator || i == data.length() - 1) {
      found++;
      if (found == index + 1) {
        end = (i == data.length() - 1) ? (i + 1) : i;
        return data.substring(start, end);
      }
      start = i + 1;
    }
  }
  return "";
}

// Function to Parse GPS Response
bool parseGPSResponse(const String &response, String &lat, String &lon) {
  int startIndex = response.indexOf("+QGPSLOC:");
  if (startIndex == -1) return false;
  startIndex = response.indexOf(':', startIndex);
  if (startIndex == -1) return false;

  String dataPart = response.substring(startIndex + 1);
  dataPart.trim();
  String latitude_temp = getValue(dataPart, ',', 1);
  String longitude_temp = getValue(dataPart, ',', 2);
  latitude_temp.trim();
  longitude_temp.trim();
  if (latitude_temp.length() == 0 || longitude_temp.length() == 0) return false;
  lat = latitude_temp;
  lon = longitude_temp;
  return true;
}

// Setup Function
void setup() {
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, RXD1, TXD1);
  delay(2000);
  Serial.println("Initializing sensors...");
  dht.begin();
  ds18b20a.begin();
  ds18b20b.begin();
  Wire.begin(40, 41);
  Serial.println("Init BNO055...");
  if (!bno.begin()) {
    Serial.println("BNO055 Fail!");
    while (1);
  }
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(TURBIDITY_ANALOG_PIN, INPUT);
  modem_init();
  Serial.println("Sensor setup completed....");
  Serial.println("MQTT Connect establish...");
  sendATCommand("AT+QMTDISC=0", 2000);
  sendATCommand("AT+QMTOPEN=0,\"" + String(mqttServer) + "\"," + String(mqttPort), 5000);
  String mqttResponse = sendATCommand("AT+QMTCONN=0,\"123\",\"" + String(mqttUser) + "\",\"" + mqttPassword + "\"", 5000);
  Serial.println("MQTT established.");
}

// Loop Function
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= publishInterval) {
    publishData();
    lastPublishTime = currentMillis;
  }
  delay(10);
}

// Publish Sensor Data
void publishData() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT Read Fail");
    return;
  }

  ds18b20a.requestTemperatures();
  float dsTempA = ds18b20a.getTempCByIndex(0);
  ds18b20b.requestTemperatures();
  float dsTempB = ds18b20b.getTempCByIndex(0);

  sensors_event_t orientationData, accelData, gyroData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  String gps_response = sendATCommand("AT+QGPSLOC?", 5000);
  String latitude = "0.0";
  String longitude = "0.0";
  parseGPSResponse(gps_response, latitude, longitude);

  int rawAnalogValue = analogRead(TURBIDITY_ANALOG_PIN);
  float turbidityNTU = map(rawAnalogValue, 0, 4095, 0, 1000);

  //JSON PAYLOAD OUTPUT DATA
  String payload = "{";
  payload += "\"temperature\":";
  payload += String(temperature, 2);
  payload += ",";
  payload += "\"humidity\":";
  payload += String(humidity, 2);
  payload += ",";
  payload += "\"ds18b20a_temp\":";
  payload += String(dsTempA, 2);
  payload += ",";
  payload += "\"ds18b20b_temp\":";
  payload += String(dsTempB, 2);
  payload += ",";
  payload += "\"turbidity_raw\":" + String(rawAnalogValue) + ",";
  payload += "\"turbidity_ntu\":" + String(turbidityNTU, 2) + ",";
  payload += "\"orientation2_x\":" + String(orientationData.orientation.x, 2) + ",";
  payload += "\"orientation_y\":" + String(orientationData.orientation.y, 2) + ",";
  payload += "\"orientation_z\":" + String(orientationData.orientation.z, 2) + ",";
  payload += "\"accel_x\":" + String(accelData.acceleration.x, 2) + ",";
  payload += "\"accel_y\":" + String(accelData.acceleration.y, 2) + ",";
  payload += "\"accel_z\":" + String(accelData.acceleration.z, 2) + ",";
  payload += "\"gyro_x\":" + String(gyroData.gyro.x, 2) + ",";
  payload += "\"gyro_y\":" + String(gyroData.gyro.y, 2) + ",";
  payload += "\"gyro_z\":" + String(gyroData.gyro.z, 2);
  payload += ",";
  payload += "\"lat\":\"" + String(latitude) + "\",";
  payload += "\"lon\":\"" + String(longitude) + "\"";
  payload += "}";

  //PUBLISH SENSOR DATA TO MQTT
  sendATCommand("AT+QMTPUB=0,0,0,0,\"" + String(mqttPublishTopic) + "\"", 2000);
  SerialAT.print(payload);
  SerialAT.write(0x1A);
  Serial.println("Data Published");
}

// Modem Initialization
void modem_init() {
  sendATCommand("AT", 2000);
  sendATCommand("ATE0", 2000);
  sendATCommand("AT+CPIN?", 2000);
  sendATCommand("AT+CSQ", 2000);
  sendATCommand("AT+CREG?", 2000);
  sendATCommand("AT+CGATT?", 2000);
  sendATCommand("AT+QGPS=1,1,0,0,1", 2000);
}