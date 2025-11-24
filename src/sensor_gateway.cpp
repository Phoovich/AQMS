#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "driver/i2s.h"
#include <math.h>

// ------------------- WiFi -------------------
const char* ssid     = "Phoovich";
const char* password = "aaaaaaaa";

// ------------------- NODE DATA -------------------
float nodeTemp  = NAN;
float nodeHum   = NAN;
int   flameAnalog  = -1;
int   flameDigital = -1;
unsigned long lastNodeUpdate = 0;

// ------------------- HTTP SERVER -------------------
WebServer server(80);

// ------------------- INMP441 -------------------
#define I2S_WS  25
#define I2S_SD  33
#define I2S_SCK 32
#define I2S_PORT I2S_NUM_0
#define BUFFER_LEN 64

int16_t sBuffer[BUFFER_LEN];
float CALIBRATION_OFFSET_DB = 127.0f;

// =======================================================
// I2S Setup
// =======================================================
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
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}

// =======================================================
// HTTP: รับข้อมูลจาก SENSOR NODE
// =======================================================
void handleNodeData() {
  nodeTemp      = server.arg("temp1").toFloat();
  nodeHum       = server.arg("hum1").toFloat();
  flameAnalog   = server.arg("flameAnalog").toInt();
  flameDigital  = server.arg("flameDigital").toInt();
  lastNodeUpdate = millis();

  Serial.println("=== Received data from Sensor Node ===");
  Serial.printf("Temp1: %.2f °C\n", nodeTemp);
  Serial.printf("Hum1: %.2f %%\n", nodeHum);
  Serial.printf("Flame Analog: %d\n", flameAnalog);
  Serial.printf("Flame Digital: %d (%s)\n",
                flameDigital,
                flameDigital ? "FLAME DETECTED" : "NO FLAME");

  server.send(200, "text/plain", "Gateway received data from node");
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.print("\nGateway IP: ");
  Serial.println(WiFi.localIP());

  // HTTP routes
  server.on("/node", HTTP_GET, handleNodeData);
  server.begin();
}

// =======================================================
// LOOP
// =======================================================
void loop() {
  server.handleClient();

  size_t bytesIn = 0;
  esp_err_t result = i2s_read(
    I2S_PORT,
    &sBuffer,
    BUFFER_LEN * sizeof(int16_t),
    &bytesIn,
    portMAX_DELAY
  );

  if (result == ESP_OK && bytesIn > 0) {
    int samples_read = bytesIn / sizeof(int16_t);
    if (samples_read > 0) {

      double sumSquares = 0;
      for (int i = 0; i < samples_read; i++) {
        float sample = (float)sBuffer[i];
        sumSquares += sample * sample;
      }

      float rms = sqrtf(sumSquares / samples_read);

      float dBFS = (rms <= 0) ? -90.0 :
                   20.0f * log10f(rms / 32768.0f);

      float dBSPL = dBFS + CALIBRATION_OFFSET_DB;

      // =========================
      // PRINT ค่า SENSOR ทั้งระบบ
      // =========================
      Serial.println("\n===== GATEWAY STATUS =====");

      // 1) INMP441
      Serial.printf("INMP441: %.2f dB SPL\n", dBSPL);

      // 2) NODE data (แสดง N/A ถ้ายังไม่ส่งมา)
      Serial.printf("Node Temp : %s\n",
                    isnan(nodeTemp) ? "N/A" : String(nodeTemp, 2).c_str());
      Serial.printf("Node Hum  : %s\n",
                    isnan(nodeHum) ? "N/A" : String(nodeHum, 2).c_str());

      Serial.printf("Flame Analog : %d\n", flameAnalog);
      Serial.printf("Flame Digital: %d (%s)\n",
                    flameDigital,
                    flameDigital ? "FLAME DETECTED" : "NO FLAME");

      Serial.println("===========================\n");
    }
  }

  delay(500);
}
