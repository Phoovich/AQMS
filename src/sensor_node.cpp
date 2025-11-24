#include "esp32-hal-gpio.h"
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHT.h>

// ----------- DHT11 (KY-015) -----------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ----------- KY-026 Flame Sensor -----------
#define FLAME_SENSOR_ANALOG_PIN 36   // VP (GPIO36) อ่าน analog
#define FLAME_SENSOR_DIGITAL_PIN 15  // ขาดิจิทัล

// ====== Wi-Fi ======
const char* ssid     = "Phoovich";
const char* password = "aaaaaaaa";

// ใส่ IP ของ Gateway ตรงนี้ (ดูจาก Serial ของ Gateway)
String gatewayIP = "172.20.10.3";  // แก้ให้ตรงกับของคุณ

void setup() {
  Serial.begin(115200);

  // DHT11
  dht.begin();

  // Flame sensor
  pinMode(FLAME_SENSOR_DIGITAL_PIN, INPUT);

  Serial.println("Sensor Node starting...");
  Serial.println(" - DHT11 (KY-015)");
  Serial.println(" - KY-026 Flame sensor");

  // Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Node connected! IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  delay(2000); // อ่านทุก 2 วินาที

  // ---------- อ่าน DHT11 ----------
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // °C

  // ---------- อ่าน KY-026 ----------
  int flameAnalog  = analogRead(FLAME_SENSOR_ANALOG_PIN);
  int flameDigital = digitalRead(FLAME_SENSOR_DIGITAL_PIN);

  if (isnan(h) || isnan(t)) {
    Serial.println("อ่านค่า DHT (KY-015) ไม่สำเร็จ");
  } else {
    Serial.print("DHT11 -> Temp: ");
    Serial.print(t);
    Serial.print(" °C, Humidity: ");
    Serial.print(h);
    Serial.println(" %");
  }

  Serial.print("Flame Sensor -> Analog: ");
  Serial.print(flameAnalog);
  Serial.print(", Digital: ");
  Serial.println(flameDigital);
  // ---------- ส่ง HTTP ไป Gateway ----------
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // ส่งแบบ GET
    String url = "http://" + gatewayIP + "/node";
    url += "?temp1=" + String(t, 1);
    url += "&hum1=" + String(h, 1);
    url += "&flameAnalog=" + String(flameAnalog);
    url += "&flameDigital=" + String(flameDigital);   // 0 หรือ 1

    Serial.print("Request URL: ");
    Serial.println(url);

    http.begin(url);
    int httpCode = http.GET();

    Serial.print("HTTP Response code: ");
    Serial.println(httpCode);

    if (httpCode > 0) {
      String payload = http.getString();
      Serial.print("Gateway reply: ");
      Serial.println(payload);
    }

    http.end();
  } else {
    Serial.println("WiFi lost, reconnecting...");
    WiFi.begin(ssid, password);
  }
}
