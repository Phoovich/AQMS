#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>   // ใช้ sqrtf, log10f

// INMP441 I2S microphone pins
#define I2S_WS  25
#define I2S_SD  33
#define I2S_SCK 32

#define I2S_PORT I2S_NUM_0
#define BUFFER_LEN 64
int16_t sBuffer[BUFFER_LEN];

// ----- CONFIG การ Calibrate -----
// ตั้ง offset ตรงนี้หลังจากทำการ Calibrate แล้ว (อธิบายด้านล่าง)
float CALIBRATION_OFFSET_DB = 127.0f;   // ค่าเริ่มต้น สมมติ, เดี๋ยวพี่ปรับเองอีกที

void i2s_install() {
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
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

void setup() {
  Serial.begin(115200);
  delay(1000);

  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);

  Serial.println("Start INMP441 dB SPL meter (with calibration offset)");
}

void loop() {
  // สำหรับ Serial Plotter → ปรับช่วง dB SPL ตามที่อยากดู
  int minSPL = 40;   // dB ต่ำสุดที่แสดง
  int maxSPL = 100;  // dB สูงสุดที่แสดง

  Serial.print(minSPL);
  Serial.print(" ");
  Serial.print(maxSPL);
  Serial.print(" ");

  size_t bytesIn = 0;
  esp_err_t result = i2s_read(
    I2S_PORT,
    &sBuffer,
    BUFFER_LEN * sizeof(int16_t),
    &bytesIn,
    portMAX_DELAY
  );

  if (result == ESP_OK && bytesIn > 0) {
    int16_t samples_read = bytesIn / sizeof(int16_t);

    if (samples_read > 0) {
      double sumSquares = 0.0;

      for (int i = 0; i < samples_read; i++) {
        float sample = (float)sBuffer[i];
        sumSquares += sample * sample;
      }

      // 1) RMS ของสัญญาณ
      float rms = sqrtf(sumSquares / samples_read);

      // 2) แปลงเป็น dBFS
      float dBFS;
      if (rms <= 0.0f) {
        dBFS = -90.0f;   // ถ้าไม่มีสัญญาณ ให้ -90 dBFS เป็นค่า floor
      } else {
        float ref = 32768.0f; // full scale ของ 16-bit
        dBFS = 20.0f * log10f(rms / ref);
      }

      // 3) แปลงเป็น dB SPL โดยบวก offset จากการ Calibrate
      float dBSPL = dBFS + CALIBRATION_OFFSET_DB;

      // ถ้าอยาก debug ดูทั้ง dBFS และ dB SPL:
      // Serial.print("dBFS=");
      // Serial.print(dBFS);
      // Serial.print(" dBSPL=");

      Serial.println(dBSPL);
    }
  }

  // ปรับ delay ให้กราฟลื่นตามต้องการ
  delay(50);
}
