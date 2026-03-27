#include "laser.h"
#include <Arduino.h>

#define TX_PIN 17
#define RX_PIN 16

char boot_buffer[4]  = {0x80, 0x06, 0x03, 0x77};
char max_rate_cmd[5] = {0x80, 0x04, 0x05, 0x00, 0x77};

unsigned char data[11] = {0};
float distance = 0.0;

unsigned long lastGoodFrame = 0;

void setupDistance() {
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial1.write(max_rate_cmd, sizeof(max_rate_cmd));
  delay(50);
  Serial1.write(boot_buffer, sizeof(boot_buffer));

  lastGoodFrame = millis();
}

void updateDistance() {
  // Restart continuous mode only if data stops
  if (millis() - lastGoodFrame > 500) {
    Serial1.write(boot_buffer, 4);
    lastGoodFrame = millis();
  }

  // Ensure data exists before peek
  if (!Serial1.available()) return;

  // Sync to frame start
  while (Serial1.available() && Serial1.peek() != 0x80) {
    Serial1.read();
  }

  // Wait for full frame
  if (Serial1.available() < 11) return;

  // Read frame
  for (int i = 0; i < 11; i++) {
    data[i] = Serial1.read();
  }

  // Validate header
  if (data[0] != 0x80 || data[1] != 0x06) return;

  // Checksum
  unsigned char Check = 0;
  for (int i = 0; i < 10; i++) {
    Check += data[i];
  }
  Check = ~Check + 1;

  if (data[10] != Check) return;

  // Valid frame
  lastGoodFrame = millis();

  if (data[3] == 'E' && data[4] == 'R' && data[5] == 'R') {
    distance = -1.0;
  } else {
    distance = (data[3] - '0') * 100 +
               (data[4] - '0') * 10 +
               (data[5] - '0') * 1 +
               (data[7] - '0') * 0.1 +
               (data[8] - '0') * 0.01 +
               (data[9] - '0') * 0.001;
  }
}