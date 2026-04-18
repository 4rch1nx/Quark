// Ground Station
#include <SPI.h>
#include <LoRa.h>

#define R 3
#define G 4
#define B 5
#define O 6

unsigned long lastPkt = 0;

// FIX: non-blocking serial command buffer
//      Previously Serial.readStringUntil() would block the loop, causing LoRa
//      packets to be missed while waiting for the user to finish typing.
char cmdBuf[32];
uint8_t cmdBufIdx = 0;

void leds(bool r, bool g, bool b, bool o) {
  digitalWrite(R, r);
  digitalWrite(G, g);
  digitalWrite(B, b);
  digitalWrite(O, o);
}

// FIX: commands now carry a checksum so the rocket can verify them,
//      matching the same scheme used for telemetry packets.
void sendCommand(int cmd, int val) {
  char buf[40];
  snprintf(buf, sizeof(buf), "CMD;FF;%d;%d", cmd, val);

  uint8_t cs = 0;
  for (char *p = buf; *p; p++) cs += *p;

  LoRa.beginPacket();
  LoRa.print(buf);
  LoRa.print(';');
  LoRa.print(cs);
  LoRa.endPacket();  // blocking TX

  Serial.print("CMD_SENT;");
  Serial.print(cmd);
  Serial.print(';');
  Serial.println(val);

  // FIX: re-enter receive mode right after TX so we can catch the ACK.
  //      Without this the radio stays in standby and the ACK is lost.
  LoRa.receive();
}

void setup() {
  Serial.begin(115200);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(O, OUTPUT);
  leds(1, 1, 1, 1);
  Serial.println("LoRa Ground Station");
  Serial.println("Commands: send  <cmd>,<val>;  over serial");
  delay(500);
  leds(0, 0, 0, 0);

  if (!LoRa.begin(433E6)) {
    Serial.println("ERR:LORA_INIT");
    leds(1, 0, 0, 0);
    while (1)
      ;
  }
  leds(0, 1, 0, 0);
  Serial.println("LoRa OK");
  delay(500);
  leds(0, 0, 0, 0);

  LoRa.receive();  // start listening immediately
}

void loop() {

  // ── RECEIVE ──────────────────────────────────────────────────────────────
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    lastPkt = millis();
    leds(0, 0, 0, 1);

    char buf[220];
    int idx = 0;
    while (LoRa.available() && idx < (int)sizeof(buf) - 1) {
      buf[idx++] = (char)LoRa.read();
    }
    buf[idx] = '\0';

    // Checksum check
    char *lastSep = strrchr(buf, ';');
    if (!lastSep) {
      Serial.println("ERR:NO_CHECKSUM");
      leds(1, 0, 0, 1);
      LoRa.receive();
      return;
    }
    uint8_t rxCS = (uint8_t)atoi(lastSep + 1);
    *lastSep = '\0';
    uint8_t calcCS = 0;
    for (char *p = buf; *p; p++) calcCS += *p;
    if (calcCS != rxCS) {
      Serial.println("ERR:BAD_CHECKSUM");
      leds(1, 0, 0, 1);
      LoRa.receive();
      return;
    }

    // FIX: distinguish ACK packets from telemetry instead of letting them
    //      fall through to field-count parsing and printing ERR:FIELD_COUNT.
    if (strncmp(buf, "ACK;", 4) == 0) {
      Serial.print("ACK;");
      Serial.println(buf + 4);  // e.g. "ACK;2"
      leds(0, 1, 0, 0);
      delay(50);
      leds(0, 0, 0, 0);
      LoRa.receive();
      return;
    }

    // --- Telemetry packet ---
    char temp[220];
    strcpy(temp, buf);

    char *fields[20];
    uint8_t count = 0;
    char *token = strtok(temp, ";");
    while (token && count < 20) {
      fields[count++] = token;
      token = strtok(NULL, ";");
    }

    if (count >= 17) {
      float altitude = atoi(fields[2]) / 100.0f;
      float voltage = atoi(fields[12]) / 100.0f;
      float vel = atoi(fields[14]) / 100.0f;
      float apogee = atoi(fields[15]) / 100.0f;

      // teamid;time;alt;alx;aly;alz;heading;status;pyro;temp;pressure;hum;volt;init;vel;apogee;pktid;rssi
      Serial.print(fields[0]);
      Serial.print(';');
      Serial.print(fields[1]);
      Serial.print(';');
      Serial.print(altitude, 2);
      Serial.print(';');
      Serial.print(fields[3]);
      Serial.print(';');
      Serial.print(fields[4]);
      Serial.print(';');
      Serial.print(fields[5]);
      Serial.print(';');
      Serial.print(fields[6]);
      Serial.print(';');
      Serial.print(fields[7]);
      Serial.print(';');
      Serial.print(fields[8]);
      Serial.print(';');
      Serial.print(fields[9]);
      Serial.print(';');
      Serial.print(fields[10]);
      Serial.print(';');
      Serial.print(fields[11]);
      Serial.print(';');
      Serial.print(voltage, 2);
      Serial.print(';');
      Serial.print(fields[13]);
      Serial.print(';');
      Serial.print(vel, 2);
      Serial.print(';');
      Serial.print(apogee, 2);
      Serial.print(';');
      Serial.print(fields[16]);
      Serial.print(';');
      Serial.println(LoRa.packetRssi());
    } else {
      Serial.println("ERR:FIELD_COUNT");
      leds(0, 1, 0, 0);
    }

    leds(0, 0, 0, 0);
    // FIX: restore receive mode after processing a packet.
    //      LoRa.parsePacket() leaves the radio in standby after reading.
    LoRa.receive();
  }

  // Lost-link indicator
  if (millis() - lastPkt > 600) leds(1, 0, 0, 0);

  // ── SEND (non-blocking serial read) ──────────────────────────────────────
  // FIX: read one byte at a time instead of blocking with readStringUntil().
  //      This keeps the receive loop running while the user types a command.
  //      Format: send  <cmd>,<val>;   e.g.  "2,1;"  to arm pyro
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == ';') {
      // End of command — parse and send
      cmdBuf[cmdBufIdx] = '\0';
      cmdBufIdx = 0;

      char *comma = strchr(cmdBuf, ',');
      if (comma) {
        *comma = '\0';
        int cmd = atoi(cmdBuf);
        int val = atoi(comma + 1);
        sendCommand(cmd, val);
      } else {
        Serial.println("ERR:BAD_CMD_FORMAT (use cmd,val;)");
      }
    } else {
      if (cmdBufIdx < (uint8_t)sizeof(cmdBuf) - 1) {
        cmdBuf[cmdBufIdx++] = c;
      }
    }
  }
}
