/*
 * ============================================================
 * WEARABLE HEALTH MONITOR - BLE + REAL SENSOR
 * ============================================================
 * 
 * Reads REAL BPM and SpO2 from MAX30102 on wrist.
 * Sends data via BLE to phone every 3 seconds.
 * No WiFi needed - phone app handles Firebase.
 * 
 * IMPORTANT: Sensor reading is NON-BLOCKING so BLE stays alive.
 * 
 * Arduino IDE Settings:
 *   Board:              ESP32S3 Dev Module
 *   Flash Size:         4MB (32Mb)
 *   Partition Scheme:   Default 4MB with spiffs
 *   USB CDC On Boot:    Enabled
 *   USB Mode:           Hardware CDC and JTAG
 *   PSRAM:              Disabled
 *   Erase All Flash:    Enabled (first upload only)
 *   Port:               COM4
 * 
 * Wiring:
 *   MAX30102 VIN  -> 3.3V
 *   MAX30102 GND  -> GND
 *   MAX30102 SDA  -> GPIO8
 *   MAX30102 SCL  -> GPIO9
 * 
 * Libraries Required (Arduino Library Manager):
 *   1. SparkFun MAX3010x Pulse and Proximity Sensor Library
 *   (BLE is built-in, no extra library needed)
 * 
 * BLE Data Format: "BPM:72,SpO2:98"
 * BLE Device Name: "HealthMonitor_ESP32"
 * 
 * Test with "Serial Bluetooth Terminal" app on phone.
 * ============================================================
 */

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ========================
// PINS
// ========================
#define SDA_PIN  8
#define SCL_PIN  9

// ========================
// BLE UUIDs - Nordic UART Service (NUS)
// ========================
// Standard UUIDs that Serial Bluetooth Terminal recognizes
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // NUS Service
#define VITALS_CHAR_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // NUS TX (ESP32 -> Phone)
#define COMMAND_CHAR_UUID   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // NUS RX (Phone -> ESP32)

// ========================
// TIMING
// ========================
#define BLE_SEND_INTERVAL   3000   // Send vitals via BLE every 3 sec

// ========================
// SENSOR OBJECTS
// ========================
MAX30105 particleSensor;
bool sensorReady = false;

// SpO2 algorithm buffers
#define BUFFER_LENGTH 100
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];
int32_t spo2Value;
int8_t  validSPO2;
int32_t heartRateValue;
int8_t  validHeartRate;

// Non-blocking sample collection
int sampleIndex = 0;           // Current sample being collected
bool samplesReady = false;     // True when 100 samples collected

// Real-time heart rate detection (beat-by-beat)
#define HR_AVG_SIZE 4
int hrReadings[HR_AVG_SIZE];    // Last 4 HR readings for averaging
int hrReadIndex = 0;
int hrTotal = 0;
int hrAverage = 0;
long lastBeat = 0;              // Timestamp of last detected beat

// Current vitals (updated each sensor read cycle)
float currentHR = 0.0;          // Heart rate (BPM) - from algorithm + beat detect
float currentSpO2 = 0.0;        // SpO2 percentage - from algorithm
bool sensorOnWrist = false;

// ========================
// BLE OBJECTS
// ========================
BLEServer*         pServer = NULL;
BLECharacteristic* pVitalsCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
BLEAdvertising*    pAdvertising = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
unsigned long lastBleSendTime = 0;
int messageCount = 0;

// ========================
// BLE SERVER CALLBACKS
// ========================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println();
    Serial.println("========================================");
    Serial.println("  PHONE CONNECTED via BLE!");
    Serial.println("========================================");
    Serial.println();
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println();
    Serial.println("========================================");
    Serial.println("  PHONE DISCONNECTED");
    Serial.println("  Restarting advertising...");
    Serial.println("========================================");
    Serial.println();
  }
};

// ========================
// COMMAND CALLBACKS (Phone -> ESP32)
// ========================
class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    
    if (value.length() > 0) {
      Serial.print("Received from phone: ");
      Serial.println(value);
      
      if (value == "STATUS") {
        String status = "OK:HR=" + String(currentHR, 1) 
                      + ",SpO2=" + String(currentSpO2, 1) 
                      + ",Sensor=" + String(sensorOnWrist ? "ON" : "OFF")
                      + ",Uptime=" + String(millis() / 1000) + "s";
        pVitalsCharacteristic->setValue(status.c_str());
        pVitalsCharacteristic->notify();
        Serial.println("  -> Sent status");
      }
      else if (value == "PING") {
        pVitalsCharacteristic->setValue("PONG");
        pVitalsCharacteristic->notify();
        Serial.println("  -> Sent PONG");
      }
      else {
        String echo = "ECHO:" + value;
        pVitalsCharacteristic->setValue(echo.c_str());
        pVitalsCharacteristic->notify();
      }
    }
  }
};

// ========================
// SETUP
// ========================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  delay(3000);
  
  Serial.println();
  Serial.println("=============================================");
  Serial.println("  WEARABLE HEALTH MONITOR");
  Serial.println("  Real MAX30102 + BLE (Non-blocking)");
  Serial.println("=============================================");
  Serial.println();
  
  // ---- Initialize BLE FIRST (so it's ready for connections) ----
  initBLE();
  
  // ---- Initialize Sensor ----
  initSensor();
  
  Serial.println();
  Serial.println("=============================================");
  Serial.println("  SYSTEM READY!");
  Serial.println("  Sensor: " + String(sensorReady ? "OK" : "NOT FOUND"));
  Serial.println("  BLE:    Advertising as HealthMonitor_ESP32");
  Serial.println("=============================================");
  Serial.println();
  Serial.println("Connect with 'Serial Bluetooth Terminal' app.");
  Serial.println("  Menu -> Devices -> BLE tab -> SCAN");
  Serial.println("Commands: PING, STATUS");
  Serial.println();
}

// ========================
// MAIN LOOP
// ========================
void loop() {
  // ---- STEP 1: Collect ONE sensor sample (non-blocking) ----
  if (sensorReady) {
    collectOneSample();
  }
  
  // ---- STEP 2: When 100 samples ready, calculate vitals ----
  if (samplesReady) {
    processVitals();
    samplesReady = false;
    sampleIndex = 0;  // Start collecting next batch
  }
  
  // ---- STEP 3: Send via BLE if connected ----
  if (deviceConnected) {
    if (millis() - lastBleSendTime >= BLE_SEND_INTERVAL) {
      lastBleSendTime = millis();
      sendVitalsBLE();
    }
  }
  
  // ---- STEP 4: Handle BLE reconnection ----
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Advertising restarted, waiting for connection...");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
  
  delay(10);  // Let BLE stack breathe
}

// ========================
// COLLECT ONE SAMPLE (non-blocking)
// ========================
// Instead of blocking for 4 seconds to collect 100 samples,
// we collect ONE sample per loop pass. This keeps BLE alive.
void collectOneSample() {
  // Check if sensor has a sample ready
  if (particleSensor.available()) {
    uint32_t red = particleSensor.getRed();
    uint32_t ir = particleSensor.getIR();
    
    redBuffer[sampleIndex] = red;
    irBuffer[sampleIndex] = ir;
    particleSensor.nextSample();
    
    // ---- Real-time beat detection on each sample ----
    if (ir > 5000) {  // Finger/wrist is on sensor
      sensorOnWrist = true;
      if (checkForBeat(ir)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        
        float bpm = 60000.0 / delta;  // Convert ms between beats to BPM
        
        if (bpm > 40 && bpm < 180) {
          // Running average of last 4 valid readings
          hrTotal -= hrReadings[hrReadIndex];
          hrReadings[hrReadIndex] = (int)bpm;
          hrTotal += (int)bpm;
          hrReadIndex = (hrReadIndex + 1) % HR_AVG_SIZE;
          hrAverage = hrTotal / HR_AVG_SIZE;
          // Only use beat-detect HR if algorithm hasn't provided one
          if (currentHR == 0) {
            currentHR = (float)hrAverage;
          }
        }
      }
    } else {
      sensorOnWrist = false;
    }
    
    sampleIndex++;
    if (sampleIndex >= BUFFER_LENGTH) {
      samplesReady = true;  // All 100 samples collected -> calculate SpO2
    }
  } else {
    particleSensor.check();  // Trigger sensor to get next sample
  }
}

// ========================
// PROCESS VITALS (called when 100 samples are ready)
// ========================
void processVitals() {
  // Check if sensor is on skin
  long avgIR = 0;
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    avgIR += irBuffer[i];
  }
  avgIR /= BUFFER_LENGTH;
  
  if (avgIR < 5000) {
    currentHR = 0;
    currentSpO2 = 0;
    sensorOnWrist = false;
    Serial.println("Sensor NOT on wrist (IR too low)");
    return;
  }
  
  // Calculate HR and SpO2 using the algorithm
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LENGTH,
    redBuffer,
    &spo2Value, &validSPO2,
    &heartRateValue, &validHeartRate
  );
  
  // Algorithm heart rate (primary source) with spike rejection + smoothing
  if (validHeartRate == 1 && heartRateValue > 40 && heartRateValue < 150) {
    float newHR = (float)heartRateValue;
    
    if (currentHR == 0) {
      // First valid reading - accept it directly
      currentHR = newHR;
    } else {
      // Spike rejection: reject if jump > 30 BPM from current
      float diff = abs(newHR - currentHR);
      if (diff < 30) {
        // Exponential smoothing: new = 0.3 * reading + 0.7 * old
        currentHR = 0.3 * newHR + 0.7 * currentHR;
      }
      // If diff >= 30, ignore the spike
    }
  }
  // If algorithm didn't produce HR, beat-detect value (from collectOneSample) is used
  
  if (validSPO2 == 1 && spo2Value > 0 && spo2Value <= 100) {
    currentSpO2 = (float)spo2Value;
  }
  
  // Print to serial (same format as IoT-Accident-Detection repo)
  Serial.print("HR=");
  Serial.print(currentHR, 1);
  Serial.print(",SpO2=");
  Serial.print(currentSpO2, 1);
  Serial.print(",IR_avg=");
  Serial.print(avgIR);
  Serial.print(",Signal=");
  if (avgIR > 100000) Serial.print("SATURATED");
  else if (avgIR > 30000) Serial.print("STRONG");
  else if (avgIR > 10000) Serial.print("OK");
  else Serial.print("WEAK");
  Serial.println();
}

// ========================
// SEND VITALS VIA BLE
// ========================
void sendVitalsBLE() {
  messageCount++;
  
  // Format matches IoT-Accident-Detection repo: "HR=78.4,SpO2=97.2"
  String vitalsData = "HR=" + String(currentHR, 1) + ",SpO2=" + String(currentSpO2, 1);
  
  pVitalsCharacteristic->setValue(vitalsData.c_str());
  pVitalsCharacteristic->notify();
  
  Serial.print("[#");
  Serial.print(messageCount);
  Serial.print("] BLE -> ");
  Serial.print(vitalsData);
  
  if (!sensorOnWrist) {
    Serial.print("  (NO WRIST)");
  }
  Serial.println();
}

// ========================
// SENSOR INITIALIZATION
// ========================
void initSensor() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(500);
  
  Serial.println("Scanning for MAX30102...");
  bool found = false;
  int retries = 0;
  
  while (!found && retries < 5) {
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0 && addr == 0x57) {
        Serial.println("  MAX30102 found at 0x57!");
        found = true;
      }
    }
    if (!found) {
      Serial.println("  Not found, retrying...");
      delay(2000);
      retries++;
    }
  }
  
  if (!found) {
    Serial.println("  MAX30102 NOT FOUND! Check wiring.");
    Serial.println("  BLE will still work - sending BPM:0,SpO2:0");
    sensorReady = false;
    return;
  }
  
  Serial.print("Initializing MAX30102... ");
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("retrying...");
    delay(2000);
  }
  Serial.println("OK!");
  
  // Wrist-optimized settings (same as wearable_health_monitor.ino)
  byte ledBrightness = 0x1F;  // Calibrated for your wrist
  byte sampleAverage = 4;     // Average 4 samples
  byte ledMode = 2;           // Red + IR
  int sampleRate = 100;       // 100 samples/sec
  int pulseWidth = 411;       // Max resolution
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth);
  sensorReady = true;
  sampleIndex = 0;
  Serial.println("Sensor configured for wrist mode!");
}

// ========================
// BLE INITIALIZATION
// ========================
void initBLE() {
  Serial.println("Initializing BLE...");
  BLEDevice::init("HealthMonitor_ESP32");
  
  // Create server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("  BLE Server created");
  
  // Create service
  BLEService* pService = pServer->createService(SERVICE_UUID);
  Serial.println("  BLE Service created");
  
  // Vitals characteristic (ESP32 -> Phone): READ + NOTIFY
  pVitalsCharacteristic = pService->createCharacteristic(
    VITALS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pVitalsCharacteristic->addDescriptor(new BLE2902());
  pVitalsCharacteristic->setValue("Waiting...");
  Serial.println("  Vitals characteristic (READ + NOTIFY)");
  
  // Command characteristic (Phone -> ESP32): WRITE
  pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());
  Serial.println("  Command characteristic (WRITE)");
  
  // Start service
  pService->start();
  Serial.println("  Service started");
  
  // Start advertising
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  Serial.println("  Advertising started!");
}
