#include "driver/i2s.h"
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <WiFi.h>
#include <math.h>

// ============ WiFi / Firebase CONFIG =================

// 1. WiFi
#define WIFI_SSID "Phoovich"
#define WIFI_PASSWORD "aaaaaaaa"

// 2. Firebase Project
#define API_KEY "AIzaSyDypT8nwRWYF3v32JlCU8l39_v81LuVEus"
#define DATABASE_URL                                                           \
  "https://smartairesp32-default-rtdb.asia-southeast1.firebasedatabase.app/"

// 3. Firebase user login
#define USER_EMAIL "6733141121@student.chula.ac.th"
#define USER_PASSWORD "gayass1234"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ============ Global sensor values (from Node + Gateway) ============

// ข้อมูลจาก Sensor Node (ส่งมาทาง HTTP)
float g_nodeTemp = NAN;
float g_nodeHum = NAN;
int g_flameAnalog = -1;
int g_flameDigital = -1;

// ข้อมูลจากไมค์ INMP441 ที่ Gateway เอง
float g_soundDb = 0.0;

// สำหรับการยิง Firebase แบบมี interval
unsigned long lastFirebaseMillis = 0;
const unsigned long FIREBASE_INTERVAL = 2000; // ms

// สำหรับการอ่านไมค์ INMP441
unsigned long lastMicReadMillis = 0;
const unsigned long MIC_INTERVAL = 500; // ms

// =============== HTTP SERVER (รับจาก Sensor Node) ==================
#include <WebServer.h>
WebServer server(80);

// =============== INMP441 I2S Microphone CONFIG =====================

#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

#define I2S_PORT I2S_NUM_0
#define BUFFER_LEN 64
int16_t sBuffer[BUFFER_LEN];

// Calibrate offset (ปรับทีหลังได้)
float CALIBRATION_OFFSET_DB = 127.0f;

// ----------------------------------------------------
void i2s_install() {
  const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = 44100,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = BUFFER_LEN,
      .use_apll = false};
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {.bck_io_num = I2S_SCK,
                                       .ws_io_num = I2S_WS,
                                       .data_out_num = -1,
                                       .data_in_num = I2S_SD};
  i2s_set_pin(I2S_PORT, &pin_config);
}

// =============== ฟังก์ชันอ่านค่าไมค์ INMP441 =======================
void readMicOnce() {
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, BUFFER_LEN * sizeof(int16_t),
                              &bytesIn, portMAX_DELAY);

  if (result == ESP_OK && bytesIn > 0) {
    int samples_read = bytesIn / sizeof(int16_t);
    if (samples_read > 0) {

      double sumSquares = 0;
      for (int i = 0; i < samples_read; i++) {
        float sample = (float)sBuffer[i];
        sumSquares += sample * sample;
      }

      float rms = sqrtf(sumSquares / samples_read);
      float dBFS;

      if (rms <= 0.0f) {
        dBFS = -90.0f;
      } else {
        float ref = 32768.0f;
        dBFS = 20.0f * log10f(rms / ref);
      }

      float dBSPL = dBFS + CALIBRATION_OFFSET_DB;

      g_soundDb = dBSPL;

      // แสดงข้อมูลรวม (Gateway + Node)
      Serial.println("\n===== GATEWAY STATUS =====");
      Serial.printf("INMP441 (Gateway) dB SPL: %.2f\n", g_soundDb);

      Serial.print("Node Temp : ");
      if (isnan(g_nodeTemp))
        Serial.println("N/A");
      else
        Serial.printf("%.2f °C\n", g_nodeTemp);

      Serial.print("Node Hum  : ");
      if (isnan(g_nodeHum))
        Serial.println("N/A");
      else
        Serial.printf("%.2f %%\n", g_nodeHum);

      Serial.printf("Flame Analog  : %d\n", g_flameAnalog);
      Serial.printf("Flame Digital : %d (%s)\n", g_flameDigital,
                    g_flameDigital ? "FLAME DETECTED" : "NO FLAME");

      Serial.println("===========================");
    }
  }
}

// =============== HTTP Handler รับค่าจาก Sensor Node ===============
void handleNodeData() {
  // ตัวอย่าง URL จาก Node:
  // /node?temp1=27.1&hum1=60.2&flameAnalog=1234&flameDigital=1

  if (server.hasArg("temp1")) {
    g_nodeTemp = server.arg("temp1").toFloat();
  }
  if (server.hasArg("hum1")) {
    g_nodeHum = server.arg("hum1").toFloat();
  }
  if (server.hasArg("flameAnalog")) {
    g_flameAnalog = server.arg("flameAnalog").toInt();
  }
  if (server.hasArg("flameDigital")) {
    g_flameDigital = server.arg("flameDigital").toInt();
  }

  Serial.println("=== Received data from Sensor Node ===");
  Serial.printf("Temp1: %.2f °C\n", g_nodeTemp);
  Serial.printf("Hum1 : %.2f %%\n", g_nodeHum);
  Serial.printf("Flame Analog : %d\n", g_flameAnalog);
  Serial.printf("Flame Digital: %d (%s)\n", g_flameDigital,
                g_flameDigital ? "FLAME DETECTED" : "NO FLAME");

  server.send(200, "text/plain", "Gateway received data from node");
}

// =============== WiFi & Firebase =====================
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
}

void initFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("Firebase initialized.");
}

// =============== ส่งข้อมูลขึ้น Firebase =====================
void sendToFirebaseOnce() {
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready yet.");
    return;
  }

  // ใช้โครงสร้าง path เดียวกับเพื่อนของคุณ
  if (!isnan(g_nodeTemp)) {
    Firebase.RTDB.pushFloat(&fbdo, "/sensors/dht/temperature", g_nodeTemp);
  }
  if (!isnan(g_nodeHum)) {
    Firebase.RTDB.pushFloat(&fbdo, "/sensors/dht/humidity", g_nodeHum);
  }

  Firebase.RTDB.pushInt(&fbdo, "/sensors/flame/analog", g_flameAnalog);
  Firebase.RTDB.pushInt(&fbdo, "/sensors/flame/digital", g_flameDigital);

  // ไม่มี photo ใน gateway ตอนนี้ ข้ามไป
  // Firebase.RTDB.setInt(&fbdo, "/sensors/photo/value", g_photoValue);

  Firebase.RTDB.pushFloat(&fbdo, "/sensors/sound/db", g_soundDb);

  Serial.println("Data sent to Firebase.");
}

// ========================== SETUP =================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32 GATEWAY (single loop + Firebase)...");

  // I2S สำหรับ INMP441
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  // WiFi & Firebase
  connectWiFi();
  initFirebase();

  // HTTP Server สำหรับรับจาก Sensor Node
  server.on("/node", HTTP_GET, handleNodeData);
  server.begin();
  Serial.println("HTTP server started on /node");
}

// ========================== LOOP ==================================
void loop() {
  // 1) ให้ Gateway รอรับ HTTP จาก Node
  server.handleClient();

  unsigned long now = millis();

  // 2) อ่านค่าไมค์ทุก ๆ MIC_INTERVAL ms
  if (now - lastMicReadMillis >= MIC_INTERVAL) {
    lastMicReadMillis = now;
    readMicOnce();
  }

  // 3) ส่งขึ้น Firebase ทุก ๆ FIREBASE_INTERVAL ms
  if (now - lastFirebaseMillis >= FIREBASE_INTERVAL) {
    lastFirebaseMillis = now;
    sendToFirebaseOnce();
  }
}
