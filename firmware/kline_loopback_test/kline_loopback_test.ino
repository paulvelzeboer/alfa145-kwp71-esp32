/*
 * kline_loopback_test - checks the ESP32's K-line UART (TX=GPIO27, RX=GPIO22)
 * by sending test bytes at 4800 baud and checking each one comes back.
 *
 * Stage 1 - ESP32 only: put a jumper wire between CN1 IO27 and CN1 IO22.
 *           Every byte must come back -> TX and RX pins work.
 *           Pull the jumper: every byte must FAIL -> proves it was the wire.
 * Stage 2 - with the ISO 9141 Click (JP1 = 3V3, 12 V on VS, no ECU, no jumper):
 *           IO27 -> Click "RX" pin, Click "TX" pin -> IO22. The L9637 echoes
 *           everything on the K-line back to RX, so every byte must come back
 *           -> the whole ESP32 -> Click -> K-line -> Click -> ESP32 path works.
 *
 * Serial Monitor at 115200. Optional: analyzer CH1 = IO27, CH2 = IO22.
 */

#define K_TX_PIN 27
#define K_RX_PIN 22
#define TEST_BAUD 4800

const uint8_t pattern[] = {0x55, 0xAA, 0x00, 0xFF, 0x10, 0x86, 0x79, 0x03};
unsigned long rounds = 0, okBytes = 0, badBytes = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  // RX idle level before the UART takes the pin: HIGH expected with the
  // jumper (TX idles high) or the Click connected (K-line idles high).
  pinMode(K_RX_PIN, INPUT);
  delay(5);
  Serial.printf("\nRX (GPIO22) idle level: %s\n", digitalRead(K_RX_PIN) ? "HIGH" : "LOW  <- expected HIGH");

  Serial2.begin(TEST_BAUD, SERIAL_8N1, K_RX_PIN, K_TX_PIN);
  Serial2.setRxFIFOFull(1);
  delay(50);
  while (Serial2.available()) Serial2.read();
}

void loop() {
  rounds++;
  String line = "round " + String(rounds) + ":";
  int roundOk = 0;

  for (uint8_t b : pattern) {
    Serial2.write(b);
    Serial2.flush();                        // wait until the byte has left TX
    int echo = -1;
    for (unsigned long t0 = millis(); millis() - t0 < 50;) {
      if (Serial2.available()) { echo = Serial2.read(); break; }
    }
    char buf[24];
    if (echo == b) {
      okBytes++; roundOk++;
      snprintf(buf, sizeof(buf), " %02X ok", b);
    } else if (echo < 0) {
      badBytes++;
      snprintf(buf, sizeof(buf), " %02X NONE", b);
    } else {
      badBytes++;
      snprintf(buf, sizeof(buf), " %02X got %02X", b, echo);
    }
    line += buf;
  }

  Serial.println(line);
  Serial.printf("  -> %s   (total ok %lu, bad %lu)\n",
                roundOk == (int)sizeof(pattern) ? "PASS" :
                roundOk == 0 ? "FAIL - nothing came back" : "PARTIAL - wiring or noise",
                okBytes, badBytes);
  delay(1000);
}
