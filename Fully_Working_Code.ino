#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <math.h>

// ====================== CUSTOMER CONFIG START ======================

// Wi-Fi
const char* WIFI_SSID     = "realme 13 Pro 5G";
const char* WIFI_PASSWORD = "x8gsiit5";

// API
const char* API_URL = "https://emergency-response-syste-ba2ce-default-rtdb.firebaseio.com/alerts.json";

// Optional API key (leave as "" if not used)
const char* API_KEY_HEADER = "X-API-Key";
const char* API_KEY_VALUE  = "YOUR_API_KEY";

// Device identity
const char* DEVICE_ID = "ESP32-ACCIDENT-01";

// -------- ACCIDENT THRESHOLDS --------
// Higher = less sensitive, lower = more sensitive.
const float IMPACT_G_THRESHOLD = 1.5;   // <<< CHANGE THIS FOR SENSITIVITY
const float GYRO_DPS_THRESHOLD = 0.0;   // 0 = ignore gyro, only G-force
const int   IMPACT_CONFIRM_SAMPLES = 2;

const uint32_t EVENT_COOLDOWN_MS = 15000;
const uint32_t SAMPLE_PERIOD_MS  = 50;

// HTTPS handling
const bool ALLOW_INSECURE_TLS = true;

// Serial print interval (combined HR+GPS+Impact line)
const uint32_t SERIAL_PRINT_MS = 1000;

// ====================== CUSTOMER CONFIG END ========================


// ====================== PIN DEFINITIONS ============================
const int NANO_RX_PIN = 16; // ESP32 RX2  <- Nano TX (D3)
const int NANO_TX_PIN = 17; // ESP32 TX2  -> Nano RX (D2) [optional]

const int GPS_RX_PIN  = 4;  // ESP32 RX1  <- GPS TX
const int GPS_TX_PIN  = 5;  // ESP32 TX1  -> GPS RX (optional)

const int MPU_SDA_PIN = 21;
const int MPU_SCL_PIN = 22;


// ====================== SERIAL PORT OBJECTS ========================
HardwareSerial NanoSerial(2); // UART2 for Nano
HardwareSerial GPSSerial(1);  // UART1 for GPS
TinyGPSPlus gps;


// ====================== MPU6050 RAW DRIVER =========================
static const uint8_t MPU_ADDR       = 0x68;
static const uint8_t PWR_MGMT_1     = 0x6B;
static const uint8_t ACCEL_CONFIG   = 0x1C;
static const uint8_t GYRO_CONFIG    = 0x1B;
static const uint8_t ACCEL_XOUT_H   = 0x3B;

// Ranges: ±16g accel, ±2000 dps gyro
static const uint8_t ACCEL_RANGE_CFG = 0x18; // ±16g
static const uint8_t GYRO_RANGE_CFG  = 0x18; // ±2000 dps

static const float ACCEL_LSB_PER_G   = 2048.0; // for ±16g
static const float GYRO_LSB_PER_DPS  = 16.4;   // for ±2000 dps

bool readMPUBurst(int16_t &ax, int16_t &ay, int16_t &az,
                  int16_t &tempRaw,
                  int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t n = Wire.requestFrom(MPU_ADDR, (uint8_t)14);
  if (n != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  tempRaw = (Wire.read() << 8) | Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);           // wake up
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(ACCEL_RANGE_CFG);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(GYRO_RANGE_CFG);
  Wire.endTransmission();

  delay(10);
}


// ====================== STATE VARIABLES ============================

// Last known HR / SpO2 from Nano
float lastHR   = 0.0f;
float lastSpO2 = 0.0f;

// Last known GPS
double lastLat = 0.0;
double lastLng = 0.0;
bool   haveGPSFix = false;

// Last impact values
float lastImpactG = 0.0f;
float lastGyroMag = 0.0f;

// Accident detection
uint32_t lastSample     = 0;
uint32_t lastEventTime  = 0;
int      impactStreak   = 0;

// Serial print timer
uint32_t lastSerialPrint = 0;


// ====================== WIFI & API HELPERS =========================
bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 12000) {
    delay(250);
  }
  return WiFi.status() == WL_CONNECTED;
}

bool postAccidentEvent(float aMagG, float gMagDps,
                       float hr, float spo2,
                       double lat, double lng, bool gpsValid) {
  if (!ensureWiFi()) {
    Serial.println("WiFi not connected. Event not sent.");
    return false;
  }

  // Make sure URL is correct and starts with https://
  String url = String(API_URL);
  Serial.print("postAccidentEvent() - URL: ");
  Serial.println(url);
  bool isHttps = url.startsWith("https://");
  Serial.print("isHttps = ");
  Serial.println(isHttps ? "true" : "false");

  // Force TLS client (testing uses setInsecure)
  WiFiClientSecure secureClient;
  secureClient.setInsecure(); // testing only — validate certs for production

  HTTPClient http;
  Serial.println("Calling http.begin(client, url) using WiFiClientSecure...");
  if (!http.begin(secureClient, url)) {
    Serial.println("HTTP begin failed (secure).");
    return false;
  }

  http.addHeader("Content-Type", "application/json");

  String payload = "{";
  payload += "\"deviceId\":\"" + String(DEVICE_ID) + "\",";
  payload += "\"event\":\"accident\",";
  payload += "\"accelMagG\":" + String(aMagG, 3) + ",";
  payload += "\"gyroMagDps\":" + String(gMagDps, 2) + ",";
  payload += "\"heartRate\":" + String(hr, 1) + ",";
  payload += "\"spo2\":" + String(spo2, 1) + ",";
  payload += "\"lat\":" + String(lat, 6) + ",";
  payload += "\"lng\":" + String(lng, 6) + ",";
  payload += "\"gpsValid\":" + String(gpsValid ? 1 : 0) + ",";
  uint64_t eventTsMs = getGPSTimestampMs(gps);
  payload += "\"timestampMs\":" + String(eventTsMs);
  payload += "}";

  Serial.print("Payload len: ");
  Serial.println(payload.length());
  Serial.print("Sending payload: ");
  Serial.println(payload);

  int code = http.POST(payload);
  String resp = http.getString();

  Serial.print("HTTP POST result code: ");
  Serial.println(code);
  Serial.print("Response body: ");
  Serial.println(resp);

  http.end();
  return (code >= 200 && code < 300);
}



// ====================== NANO HR PARSER =============================
void readNanoData() {
  // Read complete lines from Nano
  while (NanoSerial.available()) {
    String line = NanoSerial.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    // Expect: HR=78.4,SpO2=97.2
    int hrIdx = line.indexOf("HR=");
    int spIdx = line.indexOf("SpO2=");
    int commaIdx = line.indexOf(',');

    if (hrIdx != -1 && spIdx != -1 && commaIdx != -1) {
      String hrStr   = line.substring(hrIdx + 3, commaIdx);
      String spo2Str = line.substring(spIdx + 5);

      lastHR   = hrStr.toFloat();
      lastSpO2 = spo2Str.toFloat();
    }

    // You can uncomment this for debug:
    // Serial.print("Nano raw: ");
    // Serial.println(line);
  }
}

// ====================== SETTING CURRENT DATA & TIME ===============================
// Convert Gregorian date to Unix seconds (UTC)
int64_t dateTimeToUnixSeconds(int year, int month, int day, int hour, int minute, int second) {
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;

  int64_t julian = day + ((153 * m + 2) / 5) + 365LL * y + y/4 - y/100 + y/400 - 32045;
  int64_t unixDays = julian - 2440588LL;   // Days since Unix epoch
  int64_t unixSeconds = unixDays * 86400LL + hour * 3600LL + minute * 60LL + second;
  return unixSeconds;
}

// Returns real accident time from GPS (UTC)
uint64_t getGPSTimestampMs(TinyGPSPlus &gps) {
  if (gps.date.isValid() && gps.time.isValid()) {
    int64_t unixSec = dateTimeToUnixSeconds(
      gps.date.year(),
      gps.date.month(),
      gps.date.day(),
      gps.time.hour(),
      gps.time.minute(),
      gps.time.second()
    );
    return unixSec * 1000ULL;
  }

  return 0; // invalid timestamp (will show 1970)
}


// ====================== SETUP & LOOP ===============================
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C for MPU
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(100000);
  initMPU();
  Serial.println("MPU init done.");

  // UART2: Nano (HR)
  NanoSerial.begin(9600, SERIAL_8N1, NANO_RX_PIN, NANO_TX_PIN);

  // UART1: GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Try WiFi (non-blocking for system)
  if (ensureWiFi()) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi not connected yet.");
  }

  Serial.println("System ready: HR + GPS + Impact + Accident API");
  Serial.print("Impact G threshold: ");
  Serial.println(IMPACT_G_THRESHOLD);
}

void loop() {
  uint32_t now = millis();

  // ----- 1) Read heart rate from Nano -----
  readNanoData();

  // ----- 2) Feed GPS parser -----
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
  // Update last known GPS
  if (gps.location.isValid()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    haveGPSFix = true;
  } else {
    // Keep last fix, but mark validity
    haveGPSFix = false;
  }

  // ----- 3) Read MPU + Accident detection -----
  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;

    int16_t axRaw, ayRaw, azRaw, tRaw, gxRaw, gyRaw, gzRaw;
    if (readMPUBurst(axRaw, ayRaw, azRaw, tRaw, gxRaw, gyRaw, gzRaw)) {
      float axG = axRaw / ACCEL_LSB_PER_G;
      float ayG = ayRaw / ACCEL_LSB_PER_G;
      float azG = azRaw / ACCEL_LSB_PER_G;

      float gxDps = gxRaw / GYRO_LSB_PER_DPS;
      float gyDps = gyRaw / GYRO_LSB_PER_DPS;
      float gzDps = gzRaw / GYRO_LSB_PER_DPS;

      float aMagG    = sqrt(axG*axG + ayG*ayG + azG*azG);
      float gMagDps  = sqrt(gxDps*gxDps + gyDps*gyDps + gzDps*gzDps);

      lastImpactG = aMagG;
      lastGyroMag = gMagDps;

      // Accident logic
      if (now - lastEventTime >= EVENT_COOLDOWN_MS) {
        bool accelHit = (aMagG >= IMPACT_G_THRESHOLD);
        bool gyroHit  = (GYRO_DPS_THRESHOLD <= 0.0) ? true : (gMagDps >= GYRO_DPS_THRESHOLD);

        if (accelHit && gyroHit) {
          impactStreak++;
        } else {
          impactStreak = 0;
        }

        if (impactStreak >= IMPACT_CONFIRM_SAMPLES) {
          impactStreak = 0;
          lastEventTime = now;

          Serial.println("======================================");
          Serial.println("!!! ACCIDENT DETECTED - sending event !!!");
          Serial.print("ImpactG = ");
          Serial.println(aMagG, 3);
          Serial.print("GyroMag = ");
          Serial.println(gMagDps, 1);
          Serial.print("HR = ");
          Serial.println(lastHR, 1);
          Serial.print("SpO2 = ");
          Serial.println(lastSpO2, 1);
          if (haveGPSFix) {
            Serial.print("Lat = ");
            Serial.println(lastLat, 6);
            Serial.print("Lng = ");
            Serial.println(lastLng, 6);
          } else {
            Serial.println("GPS: no fix");
          }
          Serial.println("======================================");

          postAccidentEvent(aMagG, gMagDps,
                            lastHR, lastSpO2,
                            lastLat, lastLng, haveGPSFix);
        }
      } else {
        impactStreak = 0;
      }
    }
  }

  // ----- 4) Combined Serial line: HR + GPS + Impact -----
  if (now - lastSerialPrint >= SERIAL_PRINT_MS) {
    lastSerialPrint = now;

    Serial.print("HR=");
    Serial.print(lastHR, 1);
    Serial.print(",SpO2=");
    Serial.print(lastSpO2, 1);

    Serial.print(",Lat=");
    Serial.print(lastLat, 6);
    Serial.print(",Lng=");
    Serial.print(lastLng, 6);

    Serial.print(",ImpactG=");
    Serial.println(lastImpactG, 3);
  }
}