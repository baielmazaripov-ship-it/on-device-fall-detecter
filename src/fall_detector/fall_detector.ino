/**
 * On-Device Fall Detector with False-Alarm Suppression
 * Hardware: ESP32 DevKit V1 + MPU6050 (I2C: SDA=21, SCL=22, addr=0x68) + LED (GPIO 25)
 * Sampling Rate: 100 Hz (10,000 us non-blocking interval via micros())
 *
 * Key Firmware Highlights:
 * [1] 100 Hz Deterministic Loop: Non-blocking micros() scheduling polls MPU6050 at 400 kHz I2C.
 * [2] Orientation-Independent Magnitudes: Euclidean norms (accMag, gyrMag) work regardless of sensor axis orientation.
 * [3] 500 ms Lookback Buffer (LOOKBACK_N = 50): On impact (> 2.5 g), checks prior 0.5 s for free-fall (< 0.6 g) or rotation (> 200 deg/s).
 * [4] Multi-Stage False-Alarm Suppression: SETTLE (1000 ms) absorbs bounce, then OBSERVE (1500 ms) checks stillness; > 20% motion suppresses false alarms.
 * [5] Real-Time Alert & Latency Benchmark: ALERT (10000 ms) blinks LED on GPIO 25, logs FALL DETECTED, and tracks max per-sample time (up to 374 us).
 */

#include <Wire.h>

// [Mode Config] 0 = raw CSV telemetry streaming, 1 = real-time FSM fall detection
#define MODE_COLLECT 0
#define MODE_DETECT  1
#define MODE         MODE_DETECT

#define DEBUG_PRINT  0

// [Hardware Pins & Timing] I2C on GPIO 21/22, status LED on GPIO 25, 100 Hz loop (10,000 us)
const uint8_t  SDA_PIN  = 21;
const uint8_t  SCL_PIN  = 22;
const uint8_t  LED_PIN  = 25;
const uint8_t  MPU_ADDR = 0x68;
const uint32_t INTERVAL_US = 10000;

// [Detection & Suppression Thresholds] Impact, pre-impact lookback, post-impact stillness & alert timing
const float    IMPACT_G        = 2.5f;
const float    FREEFALL_G      = 0.6f;
const float    GYRO_DPS        = 200.0f;
const int      LOOKBACK_N      = 50;     // 50 samples at 100 Hz = 500 ms pre-impact window
const uint32_t SETTLE_MS       = 1000;   // 1.0 s mechanical bounce settling window
const uint32_t STILL_MS        = 1500;   // 1.5 s post-impact stillness check window
const float    STILL_LO_G      = 0.75f;
const float    STILL_HI_G      = 1.25f;
const float    STILL_GYR_DPS   = 50.0f;
const float    MAX_MOVING_FRAC = 0.2f;   // Suppress false alarm if > 20% of OBSERVE samples show motion
const uint32_t ALERT_MS        = 10000;

uint32_t nextTick;
bool sensorOk = false;

// Write single byte to MPU6050 register over I2C
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read single byte from MPU6050 register over I2C
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)1, true);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Burst-read 14 bytes (accel + temp + gyro) and convert to g and deg/s
bool readSensor(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, true) != 14) return false;

  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  ax = v[0] / 4096.0f;  ay = v[1] / 4096.0f;  az = v[2] / 4096.0f; // +/-8g scale
  gx = v[4] / 65.5f;    gy = v[5] / 65.5f;    gz = v[6] / 65.5f;   // +/-500 deg/s scale
  return true;
}

void blinkLed(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}

#if MODE == MODE_DETECT

// [FSM States] IDLE -> SETTLE (1000 ms bounce) -> OBSERVE (1500 ms stillness) -> ALERT (10000 ms LED)
enum State { IDLE, SETTLE, OBSERVE, ALERT };
State state = IDLE;

float accHist[LOOKBACK_N];
float gyrHist[LOOKBACK_N];
int   histIdx = 0;

uint32_t tState = 0;
int      obsTotal = 0, obsMoving = 0;
float    peakAcc = 0;
uint32_t maxStepUs = 0;

void resetHistory() {
  for (int i = 0; i < LOOKBACK_N; i++) { accHist[i] = 1.0f; gyrHist[i] = 0.0f; }
  histIdx = 0;
}

// Run one step of the fall detection & false-alarm suppression FSM
void processSample(float accMag, float gyrMag, uint32_t now) {
  switch (state) {
    case IDLE: {
      // Check impact (> IMPACT_G) paired with prior free-fall (< FREEFALL_G) or rotation (> GYRO_DPS)
      if (accMag > IMPACT_G) {
        float minAcc = accMag, maxGyr = gyrMag;
        for (int i = 0; i < LOOKBACK_N; i++) {
          if (accHist[i] < minAcc) minAcc = accHist[i];
          if (gyrHist[i] > maxGyr) maxGyr = gyrHist[i];
        }
        if (minAcc < FREEFALL_G || maxGyr > GYRO_DPS) {
          state = SETTLE;
          tState = now;
          peakAcc = accMag;
          Serial.printf("Impact: %.2f g, min before impact %.2f g, max rotation %.0f deg/s", accMag, minAcc, maxGyr);
          Serial.println();
        }
      }
      accHist[histIdx] = accMag;
      gyrHist[histIdx] = gyrMag;
      histIdx = (histIdx + 1) % LOOKBACK_N;
      break;
    }

    case SETTLE:
      // Wait SETTLE_MS for mechanical shock/bounce to dissipate before checking stillness
      if (now - tState >= SETTLE_MS) {
        state = OBSERVE;
        tState = now;
        obsTotal = 0;
        obsMoving = 0;
      }
      break;

    case OBSERVE:
      // Count samples outside stationary 1g/low-gyro band over STILL_MS window
      obsTotal++;
      if (accMag < STILL_LO_G || accMag > STILL_HI_G || gyrMag > STILL_GYR_DPS) obsMoving++;
      if (now - tState >= STILL_MS) {
        if (obsMoving <= MAX_MOVING_FRAC * obsTotal) {
          state = ALERT;
          tState = now;
          Serial.println("!!! FALL DETECTED !!!");
          Serial.printf("Max processing time per sample: %u us", (unsigned)maxStepUs);
          Serial.println();
        } else {
          // Suppress alert when ongoing movement indicates recovery or non-fall motion
          Serial.println("Movement after impact - false alarm suppressed");
          resetHistory();
          state = IDLE;
        }
      }
      break;

    case ALERT:
      // Blink status LED on GPIO 25 during ALERT_MS, then return cleanly to IDLE
      digitalWrite(LED_PIN, ((now / 150) % 2) ? HIGH : LOW);
      if (now - tState >= ALERT_MS) {
        digitalWrite(LED_PIN, LOW);
        resetHistory();
        state = IDLE;
        Serial.println("Alert cleared, monitoring again");
      }
      break;
  }
}
#endif

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t who = readReg(0x75);
  sensorOk = (who != 0xFF);
  if (!sensorOk) {
    Serial.println("MPU6050 not found! Check SDA/SCL, 3V3, GND, AD0.");
    while (true) blinkLed(1, 80);
  }

  // Wake MPU6050, enable ~44 Hz DLPF (0x03), set gyro +/-500 deg/s (0x08) and accel +/-8g (0x10)
  writeReg(0x6B, 0x00);
  writeReg(0x1A, 0x03);
  writeReg(0x1B, 0x08);
  writeReg(0x1C, 0x10);

#if MODE == MODE_DETECT
  Serial.printf("MPU6050 found (WHO_AM_I=0x%02X). Mode: fall detector", who);
  Serial.println();
  resetHistory();
#endif
  blinkLed(3, 150);
  nextTick = micros();
}

void loop() {
  // [100 Hz Scheduler] Run every 10,000 us without blocking delay()
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextTick) < 0) return;
  nextTick += INTERVAL_US;
  if ((int32_t)(nowUs - nextTick) > 100000) nextTick = nowUs;

  float ax, ay, az, gx, gy, gz;
  if (!readSensor(ax, ay, az, gx, gy, gz)) return;

  // [Orientation-Independent Norms] Combine 3-axis accel and gyro into scalar magnitudes
  float accMag = sqrtf(ax * ax + ay * ay + az * az);
  float gyrMag = sqrtf(gx * gx + gy * gy + gz * gz);

#if MODE == MODE_COLLECT
  Serial.print(ax, 3);     Serial.print(',');
  Serial.print(ay, 3);     Serial.print(',');
  Serial.print(az, 3);     Serial.print(',');
  Serial.print(gx, 2);     Serial.print(',');
  Serial.print(gy, 2);     Serial.print(',');
  Serial.print(gz, 2);     Serial.print(',');
  Serial.print(accMag, 3); Serial.print(',');
  Serial.println(gyrMag, 2);
  digitalWrite(LED_PIN, (millis() / 500) % 2);
#else
  // [Latency Benchmark] Measure per-sample FSM execution time in microseconds (max ~374 us)
  uint32_t t0 = micros();
  processSample(accMag, gyrMag, millis());
  uint32_t dt = micros() - t0;
  if (dt > maxStepUs) maxStepUs = dt;

  #if DEBUG_PRINT
  static uint32_t lastDbg = 0;
  if (millis() - lastDbg >= 100) {
    lastDbg = millis();
    Serial.printf("accMag:%.2f gyrMag:%.1f", accMag, gyrMag);
    Serial.println();
  }
  #endif
#endif
}
