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
bool pyro1_armed = false;
bool pyro1_state = false;
bool pyro1_fired = false;
float max_vel = 0;
float start_alt;
float apogee = 0;
float voltage = 0;
float tmp = 0;
float r1 = 29800;
float r2 = 7490;
float min_fire_alt = 20.0;
float water_alt;

const uint8_t ALT_HISTORY = 8;
float altBuffer[ALT_HISTORY];
uint8_t altIndex = 0;
unsigned long altTime[ALT_HISTORY];
bool altFull = false;

File logFile;
unsigned long lastLog = 0;
const unsigned long LOG_INT = 50;  // 20 Hz
const uint8_t checkCommandsMaxTime = 100;

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
  float heading = atan2((float)x, (float)z) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

float estimateVerticalVelocity() {
  uint8_t count = altFull ? ALT_HISTORY : altIndex;
  if (count < 3) return 0.0f;
  float sumT = 0;
  float sumH = 0;
  float sumTT = 0;
  float sumTH = 0;
  unsigned long t0 = altTime[0];
  for (uint8_t i = 0; i < count; i++) {
    float t = (altTime[i] - t0) / 1000.0f;  // seconds
    float h = altBuffer[i];                 // meters
    sumT += t;
    sumH += h;
    sumTT += t * t;
    sumTH += t * h;
  }
  float denom = count * sumTT - sumT * sumT;
  if (denom == 0) return 0.0f;
  float slope = (count * sumTH - sumT * sumH) / denom;
  return slope;
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
  while (LoRa.available() && i < (int)sizeof(buf) - 1) {
    buf[i++] = LoRa.read();
  }
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
  token = strtok(buf, ";");   // CMD
  token = strtok(NULL, ";");  // team ID — ignored for now
  token = strtok(NULL, ";");  // cmd number
  if (!token) return;
  int cmd = atoi(token);

  token = strtok(NULL, ";");  // value
  if (!token) return;
  int val = atoi(token);

  // === COMMANDS ===
  if (cmd == 2) {
    if (val == 2 && pyro1_armed) {
      pyro1_state = true;
    } else {
      pyro1_armed = (bool)val;
    }
  }
  if (cmd == 3) {
    ready = (bool)val;
    pyro1_armed = (bool)val;
    start_alt = water_alt;
  }
  if (cmd == 5) {
    start_alt = water_alt;
  }

  sendAck(cmd);
}

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
  mpuOK = mpu.testConnection();
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
      logFile.println(F("teamid,time,alt,alx,aly,alz,grx,gry,grz,heading,ready,launched,hitapg,landed,armed,state,fired,temp,prs,hum,voltage,init,vel,maxvel,apogee,logid"));
      logFile.flush();
    }
  }

  setLED(1, 1, 1);
  delay(1000);
  float start_prs = bme.readPressure();
  start_alt = pressureToAltitude(start_prs);
}

void loop() {
  unsigned long now = millis();

  // === READ SENSORS ===
  float pressure_pa = bme.readPressure();
  water_alt = pressureToAltitude(pressure_pa);
  float altitude = water_alt - start_alt;
  if (altitude > apogee) apogee = altitude;
  int8_t temperature = bme.readTemperature();
  uint8_t humidity = bme.readHumidity();

  altBuffer[altIndex] = altitude;
  altTime[altIndex] = now;
  altIndex = (altIndex + 1) % ALT_HISTORY;
  if (altIndex == 0) altFull = true;

  float vel = estimateVerticalVelocity();
  if (vel > max_vel) max_vel = vel;

  int16_t alx, aly, alz, grx, gry, grz;
  mpu.getMotion6(&alx, &aly, &alz, &grx, &gry, &grz);

  int16_t mx, my, mz;
  readQMC(&mx, &my, &mz);
  uint8_t heading = (uint8_t)getHeadingQMC(mx, my, mz);

  int v_raw = analogRead(S_RAW);
  tmp = (v_raw * 5.0) / 1024.0;
  voltage = (tmp / (r2 / (r1 + r2))) + 0.35;

  // === Status bytes ===
  status_code = 0;
  if (ready) status_code |= (1 << 0);
  if (launched) status_code |= (1 << 1);
  if (hit_apogee) status_code |= (1 << 2);
  if (landed) status_code |= (1 << 3);

  pyro1_code = 0;
  if (pyro1_armed) pyro1_code |= (1 << 0);
  if (pyro1_state) pyro1_code |= (1 << 1);
  if (pyro1_fired) pyro1_code |= (1 << 2);

  // === FLIGHT LOGIC ===

  // launched
  if (ready && !launched && (aly > 2.0f || altitude > 2)) {
    launched = true;
  }
  if (launched && altitude > min_fire_alt && !alt_threshold) {
    alt_threshold = true;
  }
  // apogee
  if (launched && !hit_apogee && (vel < -0.3f || apogee - altitude > 0.3)) {
    hit_apogee = true;
    if (alt_threshold) {
      pyro1_state = true;
    }
  }
  // landed
  static unsigned long landTime = 0;
  if (hit_apogee && !landed && abs(vel) < 1.0f) {
    if (landTime == 0) landTime = now;
    if (now - landTime > 2000) {
      landed = true;
      pyro1_armed = false;
    }
  } else {
    landTime = 0;
  }

  static unsigned long pyro1_fire_time = 0;
  if (pyro1_state) {
    if (pyro1_fire_time == 0) {
      pyro1_fire_time = millis();
      digitalWrite(PYRO_PIN, HIGH);
    }
    if (millis() - pyro1_fire_time >= 750) {
      digitalWrite(PYRO_PIN, LOW);
      pyro1_state = false;
      pyro1_fired = true;
      pyro1_fire_time = 0;
    }
  }

  // === SD LOG ===
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
    logFile.print(pyro1_armed);
    logFile.print(',');
    logFile.print(pyro1_state);
    logFile.print(',');
    logFile.print(pyro1_fired);
    logFile.print(',');
    logFile.print(temperature);
    logFile.print(',');
    logFile.print(pressure_pa, 1);
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

  // === TELEMETRY PACKET ===
  int16_t alt_i = (int16_t)(altitude * 100.0f);
  int16_t volt_i = (int16_t)(voltage * 100.0f);
  int16_t vel_i = (int16_t)(vel * 100.0f);
  int16_t apo_i = (int16_t)(apogee * 100.0f);
  uint32_t pressure_i = (uint32_t)pressure_pa;

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
  while (millis() - t < checkCommandsMaxTime) {
    checkCommands();
  }

  setLED(1, 0, 1);
}
