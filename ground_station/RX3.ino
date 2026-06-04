/*
 MEEE Sensor Receiver - ESP-NOW
 Receives data from transmitter and prints to Serial
 Supports pitch & roll from LSM6DSO
 Button on RX sends solenoid open/close command to TX
*/

#include <esp_now.h>
#include <WiFi.h>
#include <TimeLib.h>   // For hour/minute/second from unix timestamp

// ================= CONFIG =========================
#define PIN_BUTTON    D3   // Button pin (pulls LOW when pressed)
#define DEBOUNCE_MS   50   // Debounce time

// MAC address of the TX (transmitter)
static const uint8_t TX_MAC[] = { 0xDC, 0xDA, 0x0C, 0x21, 0x02, 0xA4 };  
// ================= STRUCTS ========================
typedef struct struct_message {
  uint32_t timestamp;
  float bmp_temp, bmp_pressure;
  float dht_temp, dht_humidity;
  float ax, ay, az, gx, gy, gz;
  float pitch, roll;
  float gps_lat, gps_lng, gps_alt;
  uint8_t gps_sats;
} struct_message;

// Command packet sent FROM RX -> TX
typedef struct struct_command {
  uint8_t solenoid;   // 1 = open, 0 = close
} struct_command;

struct_message incomingData;
int packetCount = 0;
unsigned long startTime = 0;

esp_now_peer_info_t txPeer;

// ================= BUTTON STATE ===================
bool solenoidState = false;       // current commanded state
bool lastButtonPhysical = HIGH;   // last raw read
bool buttonState = HIGH;          // debounced state
unsigned long lastDebounce = 0;

// ================= ESP-NOW CALLBACKS ==============
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("[CMD] Solenoid command send: %s\n",
                status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAILED");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  memcpy(&incomingData, data, sizeof(incomingData));
  packetCount++;

  if (startTime == 0) startTime = millis();

  Serial.println("=== PACKET #" + String(packetCount) + " ===");

  if (incomingData.timestamp > 946684800UL && incomingData.timestamp < 4102444800UL) {
    time_t t = incomingData.timestamp;
    char timeStr[9];
    sprintf(timeStr, "%02d:%02d:%02d", hour(t), minute(t), second(t));
    Serial.printf("Time: %s (unix=%lu)\n", timeStr, (unsigned long)incomingData.timestamp);
  } else {
    Serial.printf("Time (fallback/raw): %lu\n", (unsigned long)incomingData.timestamp);
  }

  Serial.printf("BMP: T=%.1f°C P=%.0f hPa\n", incomingData.bmp_temp, incomingData.bmp_pressure);
  Serial.printf("DHT: T=%.1f°C H=%.1f%%\n", incomingData.dht_temp, incomingData.dht_humidity);
  Serial.printf("IMU: A(%.2f, %.2f, %.2f) G(%.2f, %.2f, %.2f)\n",
                incomingData.ax, incomingData.ay, incomingData.az,
                incomingData.gx, incomingData.gy, incomingData.gz);
  Serial.printf("Orientation: Pitch=%.1f° Roll=%.1f°\n",
                incomingData.pitch, incomingData.roll);

  if (incomingData.gps_sats > 0) {
    Serial.printf("GPS: %.6f°N %.6f°E %.1f m (%d sats)\n",
                  incomingData.gps_lat, incomingData.gps_lng,
                  incomingData.gps_alt, incomingData.gps_sats);
  } else {
    Serial.println("GPS: No fix");
  }

  Serial.println("====================================");
}

// ================= SEND SOLENOID CMD ==============
void sendSolenoidCommand(bool open) {
  struct_command cmd;
  cmd.solenoid = open ? 1 : 0;
  esp_err_t err = esp_now_send(TX_MAC, (uint8_t*)&cmd, sizeof(cmd));
  if (err != ESP_OK) Serial.printf("[ERR] Command send error: %d\n", err);
  Serial.printf("[CMD] Solenoid -> %s\n", open ? "OPEN" : "CLOSE");
}

// ================= STATUS =========================
void printStatus() {
  if (startTime == 0) return;
  unsigned long uptime = millis() - startTime;
  float rate = uptime ? (packetCount * 1000.0f / uptime) : 0.0f;
  Serial.printf("[STATUS] %d pkts | %.2f Hz | Uptime: %lus | Solenoid: %s\n",
                packetCount, rate, uptime / 1000,
                solenoidState ? "OPEN" : "CLOSED");
  Serial.println("------------------------------------");
}

// ================= SETUP ==========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIN_BUTTON, INPUT_PULLUP);  // Button wired: pin -> button -> GND

  Serial.print("MY MAC: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Register TX as a peer so we can send commands to it
  memcpy(txPeer.peer_addr, TX_MAC, 6);
  txPeer.channel = 1;
  txPeer.encrypt = false;
  if (esp_now_add_peer(&txPeer) != ESP_OK) {
    Serial.println("[ERR] Failed to add TX peer. Check TX_MAC.");
  }

  Serial.println("RECEIVER READY - Waiting for sensor data...");
  Serial.println("Press button on D2 to toggle solenoid on TX side.");
}

// ================= LOOP ===========================
void loop() {
  // --- Debounced button read ---
  bool reading = digitalRead(PIN_BUTTON);

  if (reading != lastButtonPhysical) {
    lastDebounce = millis();
    lastButtonPhysical = reading;
  }

  if ((millis() - lastDebounce) > DEBOUNCE_MS) {
    // Detect falling edge (press)
    if (reading == LOW && buttonState == HIGH) {
      solenoidState = !solenoidState;       // toggle
      sendSolenoidCommand(solenoidState);
    }
    buttonState = reading;
  }

  // --- Status print every 30s ---
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 30000) {
    printStatus();
    lastStatus = millis();
  }

  delay(10);  // reduced from 100ms for button responsiveness
}
