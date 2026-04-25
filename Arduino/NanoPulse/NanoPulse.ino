// Rocket
#include <Wire.h>
#include <SPI.h>
#include <SD_fix.h>
#include <LoRa.h>
#include <GyverBME280.h>
#include <I2Cdev.h>
#include <MPU6050.h>

// === PINS ===
#define LED_R A1
#define LED_G A0
#define LED_B A2
#define PYRO_PIN 3
#define SD_CS 4
#define LORA_DIO0 2
#define LORA_RST 9
#define LORA_NSS 10
#define S_RAW A3

// === SENSORS ===
#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define BME280_ADDR 0x76
GyverBME280 bme;
MPU6050 mpu;

// === BURNOUT THRESHOLDS ===
// MPU6050 at ±2 g: 16 384 LSB/g.
// At rest the vertical (Y) axis reads ≈ 16 384 (gravity reaction).
// During motor burn it reads well above that; in free-fall it drops to ≈ 0.
//
// Tune these to your motor's thrust-to-weight ratio:
//   BOOST_THRESHOLD    : raw ADU above which we declare motor burning    (1.25 g)
//   BURNOUT_THRESHOLD  : raw ADU below which we declare motor burnt out  (1.05 g)
//   *_CONFIRM_CNT      : consecutive 20 Hz samples required (≈ 150 ms each)
#define BOOST_THRESHOLD 20480    // 1.25 g
#define BURNOUT_THRESHOLD 17200  // 1.05 g
#define BOOST_CONFIRM_CNT 3
#define BURNOUT_CONFIRM_CNT 3

// === FLAGS ===
uint8_t TeamID = 0xFF;
uint8_t pktId = 0;
char init_code = 0;
char status_code = 0;
char pyro1_code = 0;

bool bmeOK, mpuOK, qmcOK, sdOK, loraOK;
bool ready = false;
bool launched = false;
bool hit_apogee = false;
bool landed = false;
bool alt_threshold = false;
bool in_boost = false;
bool burnout_detected = false;
bool pyro1_armed = false;
bool pyro1_state = false;
bool pyro1_fired = false;

uint16_t pyroFireTime = 750;
float max_vel = 0;
float start_alt;
float apogee = 0;
float voltage = 0;
float r1 = 29800;
float r2 = 7490;
float min_fire_alt = 20.0f;
float water_alt;
const float g = 9.815f;

File logFile;
unsigned long lastLog = 0;
const unsigned long LOG_INT = 50;  // 20 Hz
uint8_t checkCommandsMaxTime = 100;

struct {
  float alt;        // fused relative altitude  [m]
  float vel;        // fused vertical velocity   [m/s]
  float vel_imu;    // IMU-only integrated velocity (internal)
  float last_baro;  // previous baro relative altitude
  unsigned long lastT;
  bool initialized;
} fuse;

void fusionReset(float baro_rel) {
  fuse.alt = baro_rel;
  fuse.vel = 0.0f;
  fuse.vel_imu = 0.0f;
  fuse.last_baro = baro_rel;
  fuse.lastT = millis();
  fuse.initialized = true;
}

void fusionUpdate(float baro_rel, int16_t aly) {
  if (!fuse.initialized) {
    fusionReset(baro_rel);
    return;
  }

  unsigned long now = millis();
  float dt = (now - fuse.lastT) * 0.001f;
  fuse.lastT = now;
  if (dt <= 0.0f || dt > 0.3f) dt = 0.02f;  // guard against timer glitches

  // ── Barometric velocity (finite difference) ──────────────────────
  float vel_baro = (baro_rel - fuse.last_baro) / dt;
  fuse.last_baro = baro_rel;
  vel_baro = constrain(vel_baro, -400.0f, 400.0f);

  // ── IMU net vertical acceleration ────────────────────────────────
  // Y axis points up → at rest reads +1 g. Subtracting g gives net body acc.
  float acc = ((float)aly / 16384.0f) * g - g;
  if (fabsf(acc) < 0.20f) acc = 0.0f;  // noise dead-band

  // Don't integrate IMU while on ground — prevents pre-flight drift
  if (!launched || landed) {
    fuse.vel_imu = 0.0f;
    acc = 0.0f;
  }

  // Integrate IMU velocity
  fuse.vel_imu += acc * dt;
  fuse.vel_imu = constrain(fuse.vel_imu, -500.0f, 500.0f);

  // ── Dynamic blending weights ─────────────────────────────────────
  //  Ground / landed : pure baro   (IMU drift irrelevant)
  //  Boost phase     : 90 % IMU    (baro lags badly at high speed)
  //  Coast / descent : ramp by speed 15 → 75 % IMU
  float speed = fabsf(fuse.vel);
  float imu_w;

  if (!launched || landed) {
    imu_w = 0.0f;
  } else if (in_boost) {
    imu_w = 0.90f;
  } else {
    imu_w = constrain(speed / 60.0f, 0.15f, 0.75f);
  }

  // Fuse velocity
  fuse.vel = imu_w * fuse.vel_imu + (1.0f - imu_w) * vel_baro;

  // Slowly pull IMU velocity toward baro reference to limit drift.
  // Skip during boost so baro lag doesn't contaminate the fast IMU estimate.
  if (!in_boost) {
    fuse.vel_imu = fuse.vel_imu * 0.97f + vel_baro * 0.03f;
  }

  // ── Integrate fused velocity → altitude ──────────────────────────
  fuse.alt += fuse.vel * dt;

  // Complementary correction: nudge fused altitude toward baro.
  // Small α during boost (trust IMU), larger α at other times (trust baro).
  float baro_alpha = in_boost ? 0.01f : 0.05f;
  fuse.alt = (1.0f - baro_alpha) * fuse.alt + baro_alpha * baro_rel;
}

// ═══════════════════════════════════════════════════════════════════

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, !r);
  digitalWrite(LED_G, !g);
  digitalWrite(LED_B, !b);
}

bool initQMC() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission()) return false;
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09);
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0A);
  Wire.write(0x11);
  Wire.endTransmission();
  return true;
}

void readQMC(int16_t *x, int16_t *y, int16_t *z) {
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(QMC_ADDR, 6);
  if (Wire.available() == 6) {
    *x = Wire.read() | (Wire.read() << 8);
    *y = Wire.read() | (Wire.read() << 8);
    *z = Wire.read() | (Wire.read() << 8);
  } else {
    *x = *y = *z = 0;
  }
}

float getHeadingQMC(int16_t x, int16_t y, int16_t z) {
  float h = atan2((float)x, (float)z) * 180.0f / PI;
  if (h < 0) h += 360.0f;
  return h;
}

void sendAck(int cmd) {
  char ack[30];
  snprintf(ack, sizeof(ack), "ACK;%d", cmd);
  uint8_t cs = 0;
  for (char *p = ack; *p; p++) cs += *p;
  LoRa.beginPacket();
  LoRa.print(ack);
  LoRa.print(';');
  LoRa.print(cs);
  LoRa.endPacket();
  LoRa.receive();
}

void checkCommands() {
  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  char buf[80];
  int i = 0;
  while (LoRa.available() && i < (int)sizeof(buf) - 1)
    buf[i++] = LoRa.read();
  buf[i] = '\0';

  char *lastSep = strrchr(buf, ';');
  if (!lastSep) return;
  uint8_t rxCS = (uint8_t)atoi(lastSep + 1);
  *lastSep = '\0';
  uint8_t calcCS = 0;
  for (char *p = buf; *p; p++) calcCS += *p;
  if (calcCS != rxCS) return;

  if (strncmp(buf, "CMD;", 4) != 0) return;

  char *token;
  token = strtok(buf, ";");   // "CMD"
  token = strtok(NULL, ";");  // team ID (ignored)
  token = strtok(NULL, ";");  // cmd number
  if (!token) return;
  int cmd = atoi(token);

  token = strtok(NULL, ";");
  if (!token) return;
  int val = atoi(token);

  if (cmd == 2) {
    if (val == 2 && pyro1_armed) pyro1_state = true;
    else pyro1_armed = (bool)val;
  }
  if (cmd == 3) {
    setLED(0, 0, 1);
    ready = (bool)val;
    pyro1_armed = (bool)val;
    start_alt = water_alt;
    fusionReset(0.0f);
    checkCommandsMaxTime = ready ? 50 : 100;
  }
  if (cmd == 5) {
    start_alt = water_alt;
    fusionReset(0.0f);
  }

  sendAck(cmd);
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);

  setLED(1, 1, 0);
  Wire.begin();
  SPI.begin();

  bmeOK = bme.begin(BME280_ADDR);

  mpu.initialize();
  mpu.setFullScaleAccelRange(0);
  mpuOK = mpu.testConnection();
  mpu.setXAccelOffset(-1052);
  mpu.setYAccelOffset(-13);
  mpu.setZAccelOffset(529);
  mpu.setXGyroOffset(141);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(-32);

  qmcOK = initQMC();
  sdOK = SD.begin(SD_CS);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(50);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  loraOK = LoRa.begin(433E6);
  if (loraOK) LoRa.setTxPower(13);

  if (bmeOK && mpuOK && qmcOK && sdOK && loraOK) {
    init_code = 0;
    setLED(0, 1, 0);
    delay(500);
  } else {
    if (!bmeOK) init_code |= (1 << 0);
    if (!mpuOK) init_code |= (1 << 1);
    if (!qmcOK) init_code |= (1 << 2);
    if (!sdOK) init_code |= (1 << 3);
    if (!loraOK) init_code |= (1 << 4);
    setLED(1, 0, 0);
    delay(1000);
  }

  if (sdOK) {
    char name[] = "F00.CSV";
    for (uint8_t i = 0; i < 100; i++) {
      name[1] = '0' + (i / 10);
      name[2] = '0' + (i % 10);
      if (!SD.exists(name)) {
        logFile = SD.open(name, FILE_WRITE);
        break;
      }
    }
    if (logFile) {
      logFile.println(F("teamid,time,alt,alx,aly,alz,grx,gry,grz,heading,"
                        "ready,launched,hitapg,landed,inboost,burnout,"
                        "armed,state,fired,temp,prs,hum,voltage,init,"
                        "vel,maxvel,apogee,logid"));
      logFile.flush();
    }
  }

  setLED(1, 0, 1);
  delay(1000);

  float start_prs = bme.readPressure();
  start_alt = pressureToAltitude(start_prs);
  fuse.initialized = false;
}

// ═══════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Read sensors ──────────────────────────────────────────────────
  float pressure = bme.readPressure();
  water_alt = pressureToAltitude(pressure);
  int8_t temperature = bme.readTemperature();
  uint8_t humidity = bme.readHumidity();

  int16_t alx, aly, alz, grx, gry, grz;
  mpu.getMotion6(&alx, &aly, &alz, &grx, &gry, &grz);

  int16_t mx, my, mz;
  readQMC(&mx, &my, &mz);
  uint8_t heading = (uint8_t)getHeadingQMC(mx, my, mz);

  int v_raw = analogRead(S_RAW);
  float tmp = (v_raw * 5.0f) / 1024.0f;
  voltage = (tmp / (r2 / (r1 + r2))) + 0.35f;

  // ── Sensor fusion ─────────────────────────────────────────────────
  float baro_rel = water_alt - start_alt;
  fusionUpdate(baro_rel, aly);

  float altitude = fuse.alt;  // fused relative altitude  [m]
  float vel = fuse.vel;       // fused vertical velocity   [m/s]

  if (altitude > apogee) apogee = altitude;
  if (vel > max_vel) max_vel = vel;

  // ── Status bytes ──────────────────────────────────────────────────
  status_code = 0;
  if (ready) status_code |= (1 << 0);
  if (launched) status_code |= (1 << 1);
  if (hit_apogee) status_code |= (1 << 2);
  if (landed) status_code |= (1 << 3);
  if (in_boost) status_code |= (1 << 4);
  if (burnout_detected) status_code |= (1 << 5);

  pyro1_code = 0;
  if (pyro1_armed) pyro1_code |= (1 << 0);
  if (pyro1_state) pyro1_code |= (1 << 1);
  if (pyro1_fired) pyro1_code |= (1 << 2);

  // ══════════════════════════════════════════
  //  FLIGHT STATE MACHINE
  // ══════════════════════════════════════════

  // 1 — Launch detect
  if (ready && !launched && (aly > 16000 || altitude > 5.0f)) {
    launched = true;
    setLED(1, 1, 0);
  }

  // 2 — Minimum altitude for pyro arming
  if (launched && altitude > min_fire_alt && !alt_threshold)
    alt_threshold = true;

  // 3 — Burnout detection (MPU-based, debounced)
  static uint8_t boost_cnt = 0;
  static uint8_t burnout_cnt = 0;

  if (launched && !burnout_detected) {
    if (!in_boost) {
      if (aly > BOOST_THRESHOLD) {
        if (++boost_cnt >= BOOST_CONFIRM_CNT) {
          in_boost = true;
          boost_cnt = 0;
          burnout_cnt = 0;
        }
      } else {
        boost_cnt = 0;
      }
    } else {
      if (aly < BURNOUT_THRESHOLD) {
        if (++burnout_cnt >= BURNOUT_CONFIRM_CNT) {
          in_boost = false;
          burnout_detected = true;
          burnout_cnt = 0;

          if (pyro1_armed && alt_threshold)
            pyro1_state = true;

          setLED(0, 1, 1);
        }
      } else {
        burnout_cnt = 0;
      }
    }
  }

  // 4 — Apogee detect
  if (launched && !hit_apogee && (vel < -0.3f || apogee - altitude > 0.3f)) {
    hit_apogee = true;
    setLED(1, 0, 0);
  }

  // 5 — Landing detect
  static unsigned long landTime = 0;
  if (hit_apogee && !landed && fabsf(vel) < 2.0f) {
    if (landTime == 0) landTime = now;
    if (now - landTime > 2000) {
      setLED(0, 1, 0);
      landed = true;
      pyro1_armed = false;
    }
  } else {
    landTime = 0;
  }

  // 6 — Pyro drive
  static unsigned long pyro1_fire_time = 0;
  if (pyro1_state) {
    if (pyro1_fire_time == 0) {
      pyro1_fire_time = millis();
      digitalWrite(PYRO_PIN, HIGH);
    }
    if (millis() - pyro1_fire_time >= pyroFireTime) {
      digitalWrite(PYRO_PIN, LOW);
      pyro1_state = false;
      pyro1_fired = true;
      pyro1_fire_time = 0;
    }
  }

  // ── SD log ────────────────────────────────────────────────────────
  static uint8_t logId = 0;
  if (logFile && (now - lastLog >= LOG_INT)) {
    logFile.print(TeamID);
    logFile.print(',');
    logFile.print(now);
    logFile.print(',');
    logFile.print(altitude, 2);
    logFile.print(',');
    logFile.print(alx);
    logFile.print(',');
    logFile.print(aly);
    logFile.print(',');
    logFile.print(alz);
    logFile.print(',');
    logFile.print(grx);
    logFile.print(',');
    logFile.print(gry);
    logFile.print(',');
    logFile.print(grz);
    logFile.print(',');
    logFile.print(heading);
    logFile.print(',');
    logFile.print(ready);
    logFile.print(',');
    logFile.print(launched);
    logFile.print(',');
    logFile.print(hit_apogee);
    logFile.print(',');
    logFile.print(landed);
    logFile.print(',');
    logFile.print(in_boost);
    logFile.print(',');
    logFile.print(burnout_detected);
    logFile.print(',');
    logFile.print(pyro1_armed);
    logFile.print(',');
    logFile.print(pyro1_state);
    logFile.print(',');
    logFile.print(pyro1_fired);
    logFile.print(',');
    logFile.print(temperature);
    logFile.print(',');
    logFile.print(pressure, 1);
    logFile.print(',');
    logFile.print(humidity);
    logFile.print(',');
    logFile.print(voltage, 2);
    logFile.print(',');
    logFile.print((int)init_code);
    logFile.print(',');
    logFile.print(vel, 2);
    logFile.print(',');
    logFile.print(max_vel, 2);
    logFile.print(',');
    logFile.print(apogee, 2);
    logFile.print(',');
    logFile.println(logId++);
    logFile.flush();
    lastLog = now;
  }

  // ── Telemetry packet ──────────────────────────────────────────────
  int16_t alt_i = (int16_t)(altitude * 100.0f);
  int16_t volt_i = (int16_t)(voltage * 100.0f);
  int16_t vel_i = (int16_t)(vel * 100.0f);
  int16_t apo_i = (int16_t)(apogee * 100.0f);
  uint32_t pressure_i = (uint32_t)pressure;

  char packet[180];
  snprintf(packet, sizeof(packet),
           "%02X;%lu;%d;%d;%d;%d;%u;%u;%u;%d;%lu;%u;%d;%u;%d;%d;%u",
           TeamID, now,
           alt_i, alx, aly, alz,
           heading, status_code, pyro1_code,
           temperature, pressure_i, humidity,
           volt_i, init_code,
           vel_i, apo_i,
           pktId++);

  uint8_t checksum = 0;
  for (char *p = packet; *p; p++) checksum += *p;

  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.print(';');
  LoRa.print(checksum);
  LoRa.endPacket();
  LoRa.receive();

  unsigned long t = millis();
  while (millis() - t < checkCommandsMaxTime)
    checkCommands();
}
