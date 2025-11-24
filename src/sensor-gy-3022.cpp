#include <Arduino.h>
#include <Wire.h>
#include <hp_BH1750.h>

hp_BH1750 sensor;   // กำหนดออบเจกต์เซนเซอร์

void setup() {
  Serial.begin(115200);
  delay(500);

  // ❗ กำหนดขา I2C ให้ ESP32
  // SDA = 21, SCL = 22 (ต้องต่อสายให้ตรง)
  Wire.begin(21, 22);

  Serial.println("Starting hp_BH1750...");

  // เริ่มต้นเซนเซอร์ที่ address 0x23 (BH1750_TO_GROUND)
  if (!sensor.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 init failed!");
  }

  sensor.calibrateTiming();
  sensor.start();
}

void loop() {
  if (sensor.hasValue()) {
    float lux = sensor.getLux();
    Serial.print("Light: ");
    Serial.print(lux);
    Serial.println(" lux");

    sensor.start();  // เริ่มอ่านครั้งใหม่
  }

  delay(50);
}
