#include "laser.h"
#include <Arduino.h>

#define TX_PIN 17
#define RX_PIN 16

char boot_buffer[4]  = {0x80, 0x06, 0x03, 0x77};
unsigned char data[11] = {0};
float distance;

void setupDistance() {
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial1.write(boot_buffer, sizeof(boot_buffer));
  delay(1000);  // wait 1 second for sensor to boot
}

int laser_index = 0;

void updateDistance() {
  laser_index++;
  while (Serial1.available() > 0) {
    if (laser_index % 5 != 0) return;
    
    if (Serial1.available() >= 11){
      for (int i = 0; i < 11; i++) {  // wait until full frame received
        data[i] = Serial1.read();
      }

      unsigned char Check = 0;
      for (int i = 0; i < 10; i++) {
        Check += data[i];
      }
      Check = ~Check + 1;

      if (data[10] == Check) {
        if (data[3] == 'E' && data[4] == 'R' && data[5] == 'R') {
          distance = -1.0;
        } else {
          distance = 0.0;
          distance = (data[3] - 0x30) * 100 +
                    (data[4] - 0x30) * 10 +
                    (data[5] - 0x30) * 1 +
                    (data[7] - 0x30) * 0.1 +
                    (data[8] - 0x30) * 0.01 +
                    (data[9] - 0x30) * 0.001;
        }
      } else {
        distance = -1.0;
      }
    }
  }
}