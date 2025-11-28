#include "driver/i2s.h"
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <WiFi.h>
#include <math.h>
#include <time.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <hp_BH1750.h>

// ============ WiFi CONFIG =================
#define WIFI_SSID     "Phoovich"
#define WIFI_PASSWORD "aaaaaaaa"

// ============ Firebase CONFIG =============
#define API_KEY        "AIzaSyDypT8nwRWYF3v32JlCU8l39_v81LuVEus"
#define DATABASE_URL   "https://smartairesp32-default-rtdb.asia-southeast1.firebasedatabase.app/"

#define USER_EMAIL     "6733141121@student.chula.ac.th"
#define USER_PASSWORD  "gayass1234"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ============ NETPIE / MQTT CONFIG =========
const char* mqtt_server   = "broker.netpie.io";
const int   mqtt_port     = 1883;
const char* mqtt_Client   = "51f7df51-fdd6-4cbd-996f-ef2ae1a04c22";   // Client ID
const char* mqtt_username = "gEHFUNG6zPXnMpBovNHv4Q9m3xJgyu6J";      // Token
const char* mqtt_password = "JK8HurWH2SwoAT9VbttrNpDGXUuYkW93";     // Secret

WiFiClient espClient;
PubSubClient mqttClient(espClient);
char netpieMsg[256];

// ============ NTP CONFIG (เวลาไทย) =========
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;  // Thailand
const int daylightOffset_sec = 0;

// ============ Global sensor values =========
// จาก Sensor Node (ส่งมาทาง HTTP)
float g_nodeTemp = NAN;
float g_nodeHum = NAN;
int   g_flameAnalog = -1;
int   g_flameDigital = -1;

// ไมค์ INMP441 ที่ Gateway
float g_soundDb = 0.0;

// BH1750 Light sensor ที่ Gateway
hp_BH1750 lightSensor;
float g_lightLux = NAN;

// สำหรับการยิง Firebase/NETPIE แบบ interval
unsigned long lastFirebaseMillis = 0;
const unsigned long FIREBASE_INTERVAL = 2000; // ms

unsigned long lastNetpieMillis = 0;
const unsigned long NETPIE_INTERVAL = 2000;   // ms

// สำหรับการอ่านไมค์ INMP441
unsigned long lastMicReadMillis = 0;
const unsigned long MIC_INTERVAL = 500; // ms

// สำหรับ BH1750
unsigned long lastLightMillis = 0;
const unsigned long LIGHT_INTERVAL = 200; // ms

// =============== HTTP SERVER (รับจาก Sensor Node) ==============
WebServer server(80);

// =============== INMP441 I2S CONFIG ============================
#define I2S_WS  25
#define I2S_SD  33
#define I2S_SCK 32

#define I2S_PORT I2S_NUM_0
#define BUFFER_LEN 64
int16_t sBuffer[BUFFER_LEN];

// Calibrate offset
float CALIBRATION_OFFSET_DB = 127.0f;

// ================== I2S Setup =========================
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

// =============== ฟังก์ชันอ่านค่าไมค์ INMP441 ===================
void readMicOnce() {
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
      float dBFS;

      if (rms <= 0.0f) {
        dBFS = -90.0f;
      } else {
        float ref = 32768.0f;
        dBFS = 20.0f * log10f(rms / ref);
      }

      float dBSPL = dBFS + CALIBRATION_OFFSET_DB;
      g_soundDb = dBSPL;

      // Debug
      Serial.println("\n===== GATEWAY STATUS =====");
      Serial.printf("INMP441 (Gateway) dB SPL: %.2f\n", g_soundDb);

      Serial.print("Node Temp : ");
      if (isnan(g_nodeTemp)) Serial.println("N/A");
      else Serial.printf("%.2f °C\n", g_nodeTemp);

      Serial.print("Node Hum  : ");
      if (isnan(g_nodeHum)) Serial.println("N/A");
      else Serial.printf("%.2f %%\n", g_nodeHum);

      Serial.printf("Flame Analog  : %d\n", g_flameAnalog);
      Serial.printf("Flame Digital : %d (%s)\n",
                    g_flameDigital,
                    g_flameDigital ? "FLAME DETECTED" : "NO FLAME");

      Serial.print("Light Lux     : ");
      if (isnan(g_lightLux)) Serial.println("N/A");
      else Serial.printf("%.2f lux\n", g_lightLux);

      Serial.println("===========================");
    }
  }
}

// =============== HTTP Handler รับค่าจาก Sensor Node =============
void handleNodeData() {
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
  Serial.printf("Flame Digital: %d (%s)\n",
                g_flameDigital,
                g_flameDigital ? "FLAME DETECTED" : "NO FLAME");
  Serial.println("======================================");

  server.send(200, "text/plain", "Gateway received data from node");
}

// =============== WiFi / Firebase =====================
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

// =============== NETPIE MQTT =========================
void initMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to NETPIE MQTT… ");
    if (mqttClient.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// =============== NTP & Timestamp ======================
void initTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Config NTP time...");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
  } else {
    Serial.println("Time synchronized with NTP.");
  }
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time in getTimestamp()");
    return "unknown_time";
  }
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buf);
}

// =============== ส่งข้อมูลขึ้น Firebase =====================
void sendToFirebaseOnce() {
  if (!Firebase.ready()) {
    Serial.println("Firebase not ready yet.");
    return;
  }

  String ts = getTimestamp();
  Serial.print("Firebase timestamp: ");
  Serial.println(ts);

  bool ok = true;

  if (!isnan(g_nodeTemp)) {
    String path = "/sensors/dht/temperature/" + ts;
    if (!Firebase.RTDB.setFloat(&fbdo, path.c_str(), g_nodeTemp)) {
      Serial.print("Error temp: "); Serial.println(fbdo.errorReason());
      ok = false;
    }
  }
  if (!isnan(g_nodeHum)) {
    String path = "/sensors/dht/humidity/" + ts;
    if (!Firebase.RTDB.setFloat(&fbdo, path.c_str(), g_nodeHum)) {
      Serial.print("Error hum: "); Serial.println(fbdo.errorReason());
      ok = false;
    }
  }
  {
    String path = "/sensors/flame/analog/" + ts;
    if (!Firebase.RTDB.setInt(&fbdo, path.c_str(), g_flameAnalog)) {
      Serial.print("Error flame analog: "); Serial.println(fbdo.errorReason());
      ok = false;
    }
  }
  {
    String path = "/sensors/flame/digital/" + ts;
    if (!Firebase.RTDB.setInt(&fbdo, path.c_str(), g_flameDigital)) {
      Serial.print("Error flame digital: "); Serial.println(fbdo.errorReason());
      ok = false;
    }
  }
  {
    String path = "/sensors/sound/db/" + ts;
    if (!Firebase.RTDB.setFloat(&fbdo, path.c_str(), g_soundDb)) {
      Serial.print("Error sound: "); Serial.println(fbdo.errorReason());
      ok = false;
    }
  }
  {
    String path = "/sensors/light/lux/" + ts;
    if (!Firebase.RTDB.setFloat(&fbdo, path.c_str(), g_lightLux)) {
      Serial.print("Error light lux: "); Serial.println(fbdo.errorReason());
      ok = false;
    }
  }

  if (ok) {
    Serial.println("Data sent to Firebase with timestamp key.");
  }
}

// =============== ส่งข้อมูลขึ้น NETPIE =====================
void sendToNetpieOnce() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  float t  = g_nodeTemp;
  float h  = g_nodeHum;
  int   fa = g_flameAnalog;
  int   fd = g_flameDigital;
  float sd = g_soundDb;
  float lx = g_lightLux;

  String data = "{ \"data\": {";
  data += "\"Temperature\":"  + String(t, 2)  + ",";
  data += "\"Humidity\":"     + String(h, 2)  + ",";
  data += "\"FlameAnalog\":"  + String(fa)    + ",";
  data += "\"FlameDigital\":" + String(fd)    + ",";
  data += "\"SoundDb\":"      + String(sd, 2) + ",";
  data += "\"LightLux\":"     + String(lx, 2);
  data += "} }";

  data.toCharArray(netpieMsg, data.length() + 1);
  mqttClient.publish("@shadow/data/update", netpieMsg);

  Serial.print("NETPIE publish: ");
  Serial.println(data);
}

// ========================== SETUP ================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32 GATEWAY (Firebase + NETPIE + BH1750)...");

  // I2S สำหรับ INMP441
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  // WiFi
  connectWiFi();

  // NTP time
  initTime();

  // Firebase
  initFirebase();

  // NETPIE MQTT
  initMQTT();

  // BH1750 Light Sensor (SDA=21, SCL=22)
  Wire.begin(21, 22);
  Serial.println("Starting BH1750...");
  if (!lightSensor.begin(BH1750_TO_GROUND)) {
    Serial.println("BH1750 init failed!");
  } else {
    Serial.println("BH1750 OK");
  }
  lightSensor.calibrateTiming();
  lightSensor.start();

  // HTTP Server สำหรับรับจาก Sensor Node
  server.on("/node", HTTP_GET, handleNodeData);
  server.begin();
  Serial.println("HTTP server started on /node");
}

// ========================== LOOP =================================
void loop() {
  server.handleClient();

  unsigned long now = millis();

  // อ่านไมค์ทุก MIC_INTERVAL
  if (now - lastMicReadMillis >= MIC_INTERVAL) {
    lastMicReadMillis = now;
    readMicOnce();
  }

  // อ่านแสง BH1750 ทุก LIGHT_INTERVAL
  if (now - lastLightMillis >= LIGHT_INTERVAL) {
    lastLightMillis = now;
    if (lightSensor.hasValue()) {
      g_lightLux = lightSensor.getLux();
      lightSensor.start();  // trigger next read
    }
  }

  // ส่ง Firebase
  if (now - lastFirebaseMillis >= FIREBASE_INTERVAL) {
    lastFirebaseMillis = now;
    sendToFirebaseOnce();
  }

  // ส่ง NETPIE
  if (now - lastNetpieMillis >= NETPIE_INTERVAL) {
    lastNetpieMillis = now;
    sendToNetpieOnce();
  }
}
