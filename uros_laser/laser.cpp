
#include "laser.h"
#include <Arduino.h>

#define TX_PIN 17
#define RX_PIN 16

char boot_buffer[4] = {0x80, 0x06, 0x03, 0x77};
unsigned char laser_data[11] = {0};
float distance = 0.0;

void setupDistance() {
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial1.write(boot_buffer, sizeof(boot_buffer));
  delay(1000); 
}

void updateDistance() {
  while (Serial1.available() >= 11) {
    if (Serial1.peek() != 0x80) {
      Serial1.read(); 
      continue;
    }

    for (int i = 0; i < 11; i++) {
      laser_data[i] = Serial1.read();
    }

    unsigned char Check = 0;
    for (int i = 0; i < 10; i++) {
      Check += laser_data[i];
    }
    Check = ~Check + 1;

    if (laser_data[10] == Check) {
      if (laser_data[3] == 'E' && laser_data[4] == 'R' && laser_data[5] == 'R') {
        distance = -1.0;
      } else {
        distance = (laser_data[3] - '0') * 100 +
                   (laser_data[4] - '0') * 10 +
                   (laser_data[5] - '0') * 1 +
                   (laser_data[7] - '0') * 0.1 +
                   (laser_data[8] - '0') * 0.01 +
                   (laser_data[9] - '0') * 0.001;
      }
    } else {
      distance = -2.0; 
    }
  }

}
