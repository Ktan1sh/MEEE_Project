/*
 MEEE Sensor Transmitter - Production Firmware v3
 BMP280, DHT22, LSM6DSO, Air530 GPS -> ESP-NOW
 GPS time only (no RTC)
 Sends raw IMU + computed pitch/roll
 Receives solenoid open/close command from RX
*/

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>

// ================= CONFIG =========================
#define DEBUG_SERIAL       1
#define TRANSMIT_INTERVAL  1000
#define DEBUG_PRINT_EVERY  10

#define PIN_DHT       D10
#define PIN_SDA       A5
#define PIN_SCL       A4
#define PIN_GPS_RX    D0
#define PIN_GPS_TX    D1
#define PIN_SOLENOID  A7 

#define ESPNOW_CHANNEL 1
static const uint8_t RECEIVER_MAC[] = { 0xE4, 0xB0, 0x63, 0xAE, 0xCF, 0x74 };

// ================= GLOBALS ========================
Adafruit_BMP280 bmp;
DHT dht(PIN_DHT, DHT22);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Adafruit_LSM6DSOX lsm6dsox;

bool bmpOk = false;
bool imuOk = false;
bool transmitReady = false;

static bool gpsHasFix = false;
static float lastGpsLat = NAN, lastGpsLng = NAN, lastGpsAlt = NAN;
static uint8_t lastGpsSats = 0;

esp_now_peer_info_t peerInfo;

// ================= DATA STRUCTS ===================
typedef struct struct_message {
  uint32_t timestamp;
  float bmp_temp, bmp_pressure;
  float dht_temp, dht_humidity;
  float ax, ay, az;
  float gx, gy, gz;
  float pitch, roll;
  float gps_lat, gps_lng, gps_alt;
  uint8_t gps_sats;
} struct_message;

// Command packet received FROM RX
typedef struct struct_command {
  uint8_t solenoid;   // 1 = open, 0 = close
} struct_command;

struct_message sensorData;

// ================= DEBUG MACROS ===================
#if DEBUG_SERIAL
  #define LOG(x) Serial.print(x)
  #define LOGLN(x) Serial.println(x)
  #define LOGF(...) Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)
  #define LOGLN(x)
  #define LOGF(...)
#endif

// ================= HELPERS ========================
void setInvalidData() {
  sensorData.timestamp = 0;
  sensorData.bmp_temp = sensorData.bmp_pressure = NAN;
  sensorData.dht_temp = sensorData.dht_humidity = NAN;
  sensorData.ax = sensorData.ay = sensorData.az = 0;
  sensorData.gx = sensorData.gy = sensorData.gz = 0;
  sensorData.pitch = sensorData.roll = 0;
  sensorData.gps_lat = sensorData.gps_lng = sensorData.gps_alt = NAN;
  sensorData.gps_sats = 0;
}

// ================= SOLENOID =======================
void setSolenoid(bool open) {
  digitalWrite(PIN_SOLENOID, open ? HIGH : LOW);
  LOGF("[SOLENOID] %s\n", open ? "OPEN" : "CLOSED");
}

// ================= IMU ============================
void lsmInit(uint8_t addr = 0x6A) {
  delay(50);
  if (!lsm6dsox.begin_I2C(addr)) {
    LOGLN("[ERR] LSM6DSO not found at 0x6A/0x6B");
    imuOk = false;
    return;
  }
  imuOk = true;
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
  LOGLN("[OK] LSM6DSO ready");
}

// ================= ESP-NOW ========================
void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    LOGF("[TX] Failed: %d\n", status);
  }
}

// Called when RX sends a command back to TX
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(struct_command)) {
    struct_command cmd;
    memcpy(&cmd, data, sizeof(cmd));
    setSolenoid(cmd.solenoid == 1);
  }
}

void initESPNOW() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    LOGLN("[FATAL] ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);   // ← listens for solenoid commands

  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    LOGLN("[FATAL] Peer add failed");
    return;
  }

  transmitReady = true;
  LOGLN("[OK] ESP-NOW ready");
}

// ================= SETUP ==========================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_SOLENOID, OUTPUT);
  setSolenoid(false);   // start closed

  setInvalidData();
  initESPNOW();

  Wire.begin(PIN_SDA, PIN_SCL);
  delay(50);

  bmpOk = bmp.begin(0x76) || bmp.begin(0x77);
  if (!bmpOk) LOGLN("[WARN] BMP280 missing");

  dht.begin();
  delay(2000);

  gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  lsmInit();

  LOGLN("[OK] System ready");
}

// ================= LOOP ==========================
void loop() {
  static unsigned long lastTransmit = 0;
  static uint32_t transmitCount = 0;
  unsigned long now = millis();

  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  if (now - lastTransmit < TRANSMIT_INTERVAL) return;
  lastTransmit = now;

  // ===== BMP280 =====
  if (bmpOk) {
    sensorData.bmp_temp = bmp.readTemperature();
    sensorData.bmp_pressure = bmp.readPressure() / 100.0f;
  }

  // ===== DHT22 =====
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) sensorData.dht_temp = t;
  if (!isnan(h)) sensorData.dht_humidity = h;

  // ===== LSM6DSO =====
  if (imuOk) {
    sensors_event_t accel, gyro, temp;
    lsm6dsox.getEvent(&accel, &gyro, &temp);

    sensorData.ax = accel.acceleration.x / 9.81f;
    sensorData.ay = accel.acceleration.y / 9.81f;
    sensorData.az = accel.acceleration.z / 9.81f;

    sensorData.gx = gyro.gyro.x * (180 / PI);
    sensorData.gy = gyro.gyro.y * (180 / PI);
    sensorData.gz = gyro.gyro.z * (180 / PI);

    sensorData.pitch = atan2(sensorData.ax, sqrt(sensorData.ay*sensorData.ay + sensorData.az*sensorData.az)) * 180.0 / PI;
    sensorData.roll  = atan2(sensorData.ay, sqrt(sensorData.ax*sensorData.ax + sensorData.az*sensorData.az)) * 180.0 / PI;
  }

  // ===== GPS =====
  if (gps.location.isValid() && gps.satellites.value() > 0) {
    lastGpsLat  = gps.location.lat();
    lastGpsLng  = gps.location.lng();
    lastGpsAlt  = gps.altitude.meters();
    lastGpsSats = gps.satellites.value();
    gpsHasFix   = true;
  }

  if (gpsHasFix) {
    sensorData.gps_lat  = lastGpsLat;
    sensorData.gps_lng  = lastGpsLng;
    sensorData.gps_alt  = lastGpsAlt;
    sensorData.gps_sats = lastGpsSats;
  }

  // ===== Timestamp =====
  if (gps.date.isValid() && gps.time.isValid()) {
    struct tm tstruct;
    tstruct.tm_year = gps.date.year() - 1900;
    tstruct.tm_mon  = gps.date.month() - 1;
    tstruct.tm_mday = gps.date.day();
    tstruct.tm_hour = gps.time.hour();
    tstruct.tm_min  = gps.time.minute();
    tstruct.tm_sec  = gps.time.second();
    sensorData.timestamp = mktime(&tstruct);
  } else {
    sensorData.timestamp = now / 1000;
  }

#if DEBUG_SERIAL
  Serial.printf("TX | T:%.1fC H:%.1f%% AX:%.2f AY:%.2f AZ:%.2f "
                "GX:%.2f GY:%.2f GZ:%.2f Pitch:%.1f Roll:%.1f "
                "GPS:%d TS:%lu\n",
                sensorData.bmp_temp,
                sensorData.dht_humidity,
                sensorData.ax, sensorData.ay, sensorData.az,
                sensorData.gx, sensorData.gy, sensorData.gz,
                sensorData.pitch, sensorData.roll,
                sensorData.gps_sats,
                sensorData.timestamp);
#endif

  // ===== ESP-NOW send =====
  if (transmitReady) {
    esp_err_t err = esp_now_send(RECEIVER_MAC, (uint8_t*)&sensorData, sizeof(sensorData));
    if (err != ESP_OK) LOGF("[ERR] Send failed: %d\n", err);
    transmitCount++;
  }

  if (DEBUG_SERIAL && transmitCount % DEBUG_PRINT_EVERY == 0) {
    LOGF("[INFO] Sent packets: %lu\n", (unsigned long)transmitCount);
  }
}
