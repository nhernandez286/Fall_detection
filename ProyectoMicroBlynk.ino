// =========================
// BLYNK CONFIG
// =========================
#define BLYNK_TEMPLATE_ID   "TMPL2d6CR9qiO"
#define BLYNK_TEMPLATE_NAME "Proyecto Nicholas"
#define BLYNK_AUTH_TOKEN    "bA5KbhrnhYFEB4AbMpSZoMKJ3j2p6t72"

#include <Wire.h>
#include <WiFi.h>
#include <MPU9250.h>
#include <math.h>
#include <BlynkSimpleEsp32.h>

// =========================
// WIFI CONFIG
// =========================
 const char* WIFI_SSID      = "NETGEAR54";
 const char* WIFI_PASSWORD  = "freshonion362";
 //const char* WIFI_SSID = "Hern1";
 //const char* WIFI_PASSWORD = "D3s3rtstorm";
 
// =========================
// PINOUT (ESP32-WROOM)
// =========================
const int SDA_PIN  = 21;
const int SCL_PIN  = 22;

const int BUZZER_PIN      = 15;   // Buzzer on GPIO15
const int STOP_BUTTON_PIN = 19;   // Button on GPIO19 (interrupt)

// Buzzer tone frequencies
const int ALERT_TONE_FREQ = 2200; // Hz, for falls + general alerts
const int CAL_TONE_FREQ   = 1500; // Hz, for calibration complete

// =========================
// I2C ADDRESSES
// =========================
#define MPU_ADDR    0x68
#define AK8963_ADDR 0x0C

// =========================
// IMU OBJECT
// =========================
MPU9250 mpu;

// =========================
// FALL DETECTION PARAMETERS
// =========================

// Dynamic orientation baseline
float dynAx = -0.04f;
float dynAy = -0.97f;
float dynAz =  0.14f;

const float BASE_ALPHA = 0.01f;   // Baseline drift speed

// Impact thresholds
const float ACC_DELTA_IMPACT = 0.55f;
const float GYRO_COUPLED_TH  = 110.0f;
const float GYRO_HARD_IMPACT = 200.0f;
const float TILT_THRESHOLD   = 50.0f;

// Internal status for logic
String fallStatus    = "OK";       // "OK", "FALL_ALGO", "FALL_MANUAL"
String displayStatus = "Upright";  // User-facing status

// Last accel sample
float lastAx = 0.0f;
float lastAy = -1.0f;
float lastAz = 0.0f;

// Magnetometer helpers
float lastHeadingDeg = 0.0f;
bool  headingInitialized = false;

// For mMag stability
float lastMMag = 0.0f;
bool  mMagInitialized = false;
const float MMAG_STABLE_DELTA = 150.0f;

// Alarm state
bool alarmActive = false;   // true while continuous alert buzzer is on

// Button interrupt / debounce
volatile bool   buttonPressed = false;
unsigned long   lastButtonHandledTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 250;

// =========================
// FUNCTION PROTOTYPES
// =========================
void IRAM_ATTR stopButtonISR();

void beep(int ms);
void calBeep(int ms);

void initMagAK8963();
bool readMagAK8963(float &mx, float &my, float &mz);

void connectWiFi();

float computeTilt(float ax, float ay, float az);
float computeHeadingDeg(float mx, float my);

void sendDataToBlynk(const String &displayStatus, unsigned long t_ms, float tiltDeg, float aMag, float gMag);

void sendFallNotification(const char* source);

void calibrateBaseline(unsigned long durationMs);

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Starting FallDetector");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);  // Button to GND, internal pull-up
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), stopButtonISR, FALLING);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (mpu.setup(0x68)) {
    Serial.println("MPU9250 connected.");
  } else {
    Serial.println("MPU9250 NOT FOUND.");
    while (1) {
      delay(1000);
    }
  }

  initMagAK8963();
  calibrateBaseline(3000);  // ~3 seconds of standing still

  connectWiFi();

  // Ready beep (short, not latched)
  beep(100);
}

// =========================
//– LOOP
// =========================
void loop() {
  Blynk.run();

  if (mpu.update()) {
    unsigned long t_ms = millis();

    // --- accel & gyro ---
    float ax   = mpu.getAccX();
    float ay   = mpu.getAccY();
    float az   = mpu.getAccZ();
    float aMag = sqrt(ax * ax + ay * ay + az * az);

    lastAx = ax;
    lastAy = ay;
    lastAz = az;

    float gx   = mpu.getGyroX();
    float gy   = mpu.getGyroY();
    float gz   = mpu.getGyroZ();
    float gMag = sqrt(gx * gx + gy * gy + gz * gz);

    // --- magnetometer ---
    float mx, my, mz;
    bool magOk = readMagAK8963(mx, my, mz);
    if (!magOk) {
      mx = my = mz = 0.0f;
    }

    float mMag = sqrt(mx * mx + my * my + mz * mz);

    float headingDeg  = 0.0f;
    float headingDelta = 0.0f;

    if (mx != 0.0f || my != 0.0f) {
      headingDeg = computeHeadingDeg(mx, my);

      if (!headingInitialized) {
        lastHeadingDeg = headingDeg;
        headingInitialized = true;
      }

      float diff = headingDeg - lastHeadingDeg;

      if (diff > 180.0f) {
        diff -= 360.0f;
      } else {
        diff += 360.0f;
      }

      headingDelta = fabs(diff);
      lastHeadingDeg = headingDeg;
    }

    // --- mMag stability check ---
    float mMagDelta = 0.0f;
    bool  magStable = true;

    if (!mMagInitialized) {
      lastMMag = mMag;
      mMagInitialized = true;
    } else {
      mMagDelta = fabs(mMag - lastMMag);
      magStable = (mMagDelta < MMAG_STABLE_DELTA);
      lastMMag  = mMag;
    }

    if (fallStatus == "OK" &&
        gMag < 20.0f &&
        fabs(aMag - 1.0f) < 0.2f &&
        magStable) {
      dynAx = (1.0f - BASE_ALPHA) * dynAx + BASE_ALPHA * ax;
      dynAy = (1.0f - BASE_ALPHA) * dynAy + BASE_ALPHA * ay;
      dynAz = (1.0f - BASE_ALPHA) * dynAz + BASE_ALPHA * az;
    }

    float tiltDeg = computeTilt(ax, ay, az);

    // --- FALL ALGORITHM (impact + tilt) ---
    float accDelta = fabs(aMag - 1.0f);

    bool impact =  (accDelta > ACC_DELTA_IMPACT && gMag > GYRO_COUPLED_TH) ||
                   (gMag > GYRO_HARD_IMPACT);

    bool bigTilt = (tiltDeg > TILT_THRESHOLD);

    if (fallStatus == "OK" && impact && bigTilt) {
      Serial.println("FALL_ALGO detected.");
      fallStatus = "FALL_ALGO";

      // Automatic fall notification
      sendFallNotification("Automatic");

      // Start continuous alarm tone, latched until reset
      if (!alarmActive) {
        tone(BUZZER_PIN, ALERT_TONE_FREQ);
        alarmActive = true;
      }
    }

    // --- BUTTON (INTERRUPT-DRIVEN) ---
    bool pressed = false;
    if (buttonPressed) {
      noInterrupts();
      pressed = buttonPressed;
      buttonPressed = false;
      interrupts();
    }

    if (pressed) {
      unsigned long now = millis();
      if (now - lastButtonHandledTime >= BUTTON_DEBOUNCE_MS) {
        lastButtonHandledTime = now;

        if (fallStatus == "OK") {
          // Manual alert (latched)
          Serial.println("Button -> FALL_MANUAL");
          fallStatus = "FALL_MANUAL";

          // Manual fall notification
          sendFallNotification("Manual");

          if (!alarmActive) {
            tone(BUZZER_PIN, ALERT_TONE_FREQ);
            alarmActive = true;
          }
        } else {
          // Reset from any fall state
          Serial.print("Button -> reset from ");
          Serial.println(fallStatus);
          fallStatus = "OK";

          if (alarmActive) {
            noTone(BUZZER_PIN);
            alarmActive = false;
          }

          // Short confirmation beep
          beep(80);
        }
      }
    }

    // --- POSTURE / USER-FACING STATUS ---
    if (fallStatus == "FALL_ALGO") {
      displayStatus = "Fall detected (automatic)";
    } else if (fallStatus == "FALL_MANUAL") {
      displayStatus = "Fall detected (manual)";
    } else {
      if (tiltDeg < 30.0f) {
        displayStatus = "Upright";
      } else {
        displayStatus = "Laying";
      }
    }

    // --- SEND DATA TO BLYNK ---
    sendDataToBlynk(displayStatus, t_ms, tiltDeg, aMag, gMag);

    // --- SERIAL DEBUG ---
    Serial.print("t=");
    Serial.print(t_ms);
    Serial.print(" fallStatus=");
    Serial.print(fallStatus);
    Serial.print(" displayStatus=");
    Serial.print(displayStatus);
    Serial.print(" tilt=");
    Serial.print(tiltDeg);
    Serial.print(" aMag=");
    Serial.print(aMag);
    Serial.print(" gMag=");
    Serial.print(gMag);
    Serial.print(" mMag=");
    Serial.print(mMag);
    Serial.print(" mMagDelta=");
    Serial.print(mMagDelta);
    Serial.print(" magStable=");
    Serial.println(magStable ? 1 : 0);
  }

  delay(20);   // ~50 Hz
}

// =========================
// FUNCTION BODIES
// =========================

void IRAM_ATTR stopButtonISR() {
  buttonPressed = true;
}

void beep(int ms) {
  tone(BUZZER_PIN, ALERT_TONE_FREQ);
  delay(ms);
  noTone(BUZZER_PIN);
}

void calBeep(int ms) {
  tone(BUZZER_PIN, CAL_TONE_FREQ);
  delay(ms);
  noTone(BUZZER_PIN);
}

void initMagAK8963() {
  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  // Enable I2C bypass so ESP32 can talk directly to AK8963
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37);  // INT_PIN_CFG
  Wire.write(0x02);  // BYPASS_EN = 1
  Wire.endTransmission();
  delay(10);

  // Optional: check WHO_AM_I
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(AK8963_ADDR, 1);
  if (Wire.available()) {
    int whoami = Wire.read();
    Serial.print("AK8963 WHO_AM_I = 0x");
    Serial.println(whoami, HEX);
  }

  // Magnetometer: 16-bit, continuous-measurement mode 2 (100 Hz)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x0A);    // CNTL1
  Wire.write(0x16);    // 16-bit, continuous mode 2
  Wire.endTransmission();
  delay(20);
}

bool readMagAK8963(float &mx, float &my, float &mz) {
  // Select first magnetometer data register (HXL)
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);

  // Request 6 bytes for X, Y, Z
  if (Wire.requestFrom(AK8963_ADDR, 6) != 6) {
    mx = my = mz = 0.0f;
    return false;
  }

  int xL = Wire.read();
  int xH = Wire.read();
  int yL = Wire.read();
  int yH = Wire.read();
  int zL = Wire.read();
  int zH = Wire.read();

  int rawX = (xH << 8) + xL;
  int rawY = (yH << 8) + yL;
  int rawZ = (zH << 8) + zL;

  mx = (float)rawX;
  my = (float)rawY;
  mz = (float)rawZ;

  return true;
}

void connectWiFi() {
  Serial.print("Connecting to WiFi/Blynk: ");
  Serial.println(WIFI_SSID);

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed.");
  }
}

float computeTilt(float ax, float ay, float az) {
  float norm     = sqrt(ax * ax + ay * ay + az * az);
  float baseNorm = sqrt(dynAx * dynAx + dynAy * dynAy + dynAz * dynAz);
  if (norm == 0 || baseNorm == 0) {
    return 0.0f;
  }

  float dot = ax * dynAx + ay * dynAy + az * dynAz;
  float c   = dot / (norm * baseNorm);
  c         = constrain(c, -1.0f, 1.0f);

  return acos(c) * 180.0f / PI;
}

float computeHeadingDeg(float mx, float my) {
  float heading = atan2(my, mx) * 180.0f / PI;
  if (heading < 0.0f) {
    heading += 360.0f;
  }
  return heading;
}

void sendDataToBlynk(const String &displayStatus, unsigned long t_ms, float tiltDeg, float aMag, float gMag) {
  if (!Blynk.connected()) {
    return;
  }

  // V1 -> aMag
  // V2 -> gMag
  // V3 -> TiltDeg
  // V4 -> Status (string)
  // V5 -> mMag (magnetometer magnitude)
  Blynk.virtualWrite(V1, aMag);
  Blynk.virtualWrite(V2, gMag);
  Blynk.virtualWrite(V3, tiltDeg);
  Blynk.virtualWrite(V4, displayStatus);
  
}

void sendFallNotification(const char* source) {
  if (!Blynk.connected()) {
    return;
  }

  String msg = String(source) + " fall detected";
  Blynk.logEvent("fall_alert", msg);
}

void calibrateBaseline(unsigned long durationMs) {
  unsigned long start = millis();
  unsigned long count = 0;
  float sumAx = 0.0f;
  float sumAy = 0.0f;
  float sumAz = 0.0f;

  while (millis() - start < durationMs) {
    if (mpu.update()) {
      float ax = mpu.getAccX();
      float ay = mpu.getAccY();
      float az = mpu.getAccZ();
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      count++;
      delay(5);
    } else {
      delay(2);
    }
  }

  if (count > 0) {
    dynAx = sumAx / count;
    dynAy = sumAy / count;
    dynAz = sumAz / count;

    Serial.print("Baseline calibrated. dynAx=");
    Serial.print(dynAx);
    Serial.print(" dynAy=");
    Serial.print(dynAy);
    Serial.print(" dynAz=");
    Serial.println(dynAz);

    calBeep(250);
  } else {
    Serial.println("Baseline calibration failed: no samples.");
  }
}
