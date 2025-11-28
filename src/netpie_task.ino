#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ====================== DHT11 (Task1) ============================
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ====================== Flame Sensor + Buzzer (Task2) ============
#define FLAME_SENSOR_ANALOG_PIN 36
#define FLAME_SENSOR_DIGITAL_PIN 15 // ใช้ GPIO 15 แทน 4
#define BUZZER_PIN 18
bool buzzerOn = false;

// ====================== Photoresistor KY-018 (Task3) =============
#define PHOTO_PIN 32

// ====================== Sound Sensor KY-037 (Task4) ==============
#define SOUND_PIN 35
const int sampleWindow = 50; // ms

// ====================== Shared Sensor Values ======================
volatile float gHumidity = NAN;
volatile float gTemperature = NAN;
volatile int   gFlameAnalog = 0;
volatile int   gFlameDigital = 0;
volatile int   gPhotoValue = 0;
volatile float gSoundDb = 0.0;

// ====================== WiFi CONFIG ===============================
const char* ssid     = "Thunzaa";
const char* password = "tuneiei123";

// ====================== NETPIE / MQTT CONFIG =====================
const char* mqtt_server   = "broker.netpie.io";
const int   mqtt_port     = 1883;
const char* mqtt_Client   = "51f7df51-fdd6-4cbd-996f-ef2ae1a04c22";   // Client ID
const char* mqtt_username = "gEHFUNG6zPXnMpBovNHv4Q9m3xJgyu6J";      // Token
const char* mqtt_password = "JK8HurWH2SwoAT9VbttrNpDGXUuYkW93";     // Secret

WiFiClient espClient;
PubSubClient client(espClient);
char msg[256];   // buffer สำหรับ JSON

// ===================== WiFi / MQTT Helpers ========================
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to NETPIE MQTT… ");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// ======================== TASK 1: DHT11 ==========================
void TaskReadDHT(void *pvParameters) {
  dht.begin();

  while (1) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
      Serial.println("Task1 - Failed to read DHT11!");
    } else {
      gHumidity = h;
      gTemperature = t;
      Serial.printf("Task1 - Humidity: %.1f %%  Temp: %.1f °C\n", h, t);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// ======================= TASK 2: Flame + Buzzer ===================
void TaskReadFlameBuzzer(void *pvParameters) {
  pinMode(FLAME_SENSOR_DIGITAL_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  while (1) {
    int analogValue = analogRead(FLAME_SENSOR_ANALOG_PIN);
    int digitalValue = digitalRead(FLAME_SENSOR_DIGITAL_PIN);

    gFlameAnalog = analogValue;
    gFlameDigital = digitalValue;

    Serial.printf("Task2 - Flame: Analog=%d Digital=%d\n", analogValue,
                  digitalValue);

    bool flameDetected = (digitalValue == HIGH); // KY-026: HIGH = พบไฟ

    if (flameDetected) {
      Serial.println("Task2 - Flame detected! Buzzer ON.");

      if (!buzzerOn) {
        tone(BUZZER_PIN, 2000); // เปิดเสียง
        buzzerOn = true;
      }

    } else {
      Serial.println("Task2 - No flame.");

      if (buzzerOn) {
        noTone(BUZZER_PIN); // ปิดเสียง
        buzzerOn = false;
      }
    }

    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

// ======================= TASK 3: Photoresistor ====================
void TaskReadPhoto(void *pvParameters) {
  while (1) {
    int sensorValue = analogRead(PHOTO_PIN);
    gPhotoValue = sensorValue;

    Serial.printf("Task3 - Photo: %d\n", sensorValue);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ======================= TASK 4: Sound / dB ======================
void TaskReadSound(void *pvParameters) {
  while (1) {
    unsigned long startMillis = millis();
    unsigned int signalMax = 0;
    unsigned int signalMin = 4095;

    while (millis() - startMillis < sampleWindow) {
      int sample = analogRead(SOUND_PIN);
      if (sample < 4096) {
        if (sample > signalMax)
          signalMax = sample;
        if (sample < signalMin)
          signalMin = sample;
      }
    }

    float peakToPeak = signalMax - signalMin;
    if (peakToPeak < 5)
      peakToPeak = 1;

    float refAmp = 20.0;
    float ratio = peakToPeak / refAmp;
    if (ratio < 0.001)
      ratio = 0.001;

    float dB = 20.0 * log10(ratio);
    float displayDb = dB + 60.0;

    static float smoothDb = 60.0;
    smoothDb = smoothDb * 0.8 + displayDb * 0.2;

    gSoundDb = smoothDb;

    Serial.printf("Task4 - Sound dB: %.1f\n", smoothDb);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ======================= TASK 5: Send to NETPIE ==================
void TaskSendToNetpie(void *pvParameters) {
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }

    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop();

    // อ่านค่า global (จะได้ snapshot ตอนนี้)
    float h  = gHumidity;
    float t  = gTemperature;
    int   fa = gFlameAnalog;
    int   fd = gFlameDigital;
    int   ph = gPhotoValue;
    float sd = gSoundDb;

    // สร้าง JSON
    String data = "{ \"data\": {";
    data += "\"Humidity\":"     + String(h, 2) + ",";
    data += "\"Temperature\":"  + String(t, 2) + ",";
    data += "\"FlameAnalog\":"  + String(fa)   + ",";
    data += "\"FlameDigital\":" + String(fd)   + ",";
    data += "\"Photo\":"        + String(ph)   + ",";
    data += "\"SoundDb\":"      + String(sd, 1);
    data += "} }";

    data.toCharArray(msg, data.length() + 1);
    client.publish("@shadow/data/update", msg);

    Serial.print("NETPIE publish: ");
    Serial.println(data);

    vTaskDelay(2000 / portTICK_PERIOD_MS); // ส่งทุก ๆ 2 วินาที
  }
}

// ========================== SETUP =================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32 Multitasking (4 Tasks + NETPIE)...");

  // WiFi + MQTT setup
  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);

  // Task1 - DHT11 (Core 0)
  xTaskCreatePinnedToCore(TaskReadDHT, "Task1_DHT", 4096, NULL, 1, NULL, 0);

  // Task2 - Flame + Buzzer (Core 1)
  xTaskCreatePinnedToCore(TaskReadFlameBuzzer, "Task2_Flame", 4096, NULL, 1,
                          NULL, 1);

  // Task3 - Photoresistor (Core 0)
  xTaskCreatePinnedToCore(TaskReadPhoto, "Task3_Photo", 4096, NULL, 1, NULL, 0);

  // Task4 - Sound Sensor (Core 1)
  xTaskCreatePinnedToCore(TaskReadSound, "Task4_Sound", 4096, NULL, 1, NULL, 1);

  // Task5 - Send NETPIE (Core 1)
  xTaskCreatePinnedToCore(TaskSendToNetpie, "Task5_NETPIE", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // ว่าง — FreeRTOS ทำงานเอง
}
