#include "laser.h"
#include "imu.h"

const unsigned long imuInterval = 10;    // 100Hz for smooth control
const unsigned long laserInterval = 50;  // 20Hz is usually plenty for distance
const unsigned long printInterval = 100; // 10Hz for the Serial Monitor

unsigned long lastImuTime = 0;
unsigned long lastLaserTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  setupDistance(); 
  setupIMU();

}

void loop(){
  unsigned long currentMillis = millis();
  updateIMU();
  if (currentMillis - lastLaserTime >= laserInterval) {
    updateDistance();
    lastLaserTime = currentMillis;
  }

  static unsigned long last = 0;
  if (currentMillis - lastPrintTime >= printInterval){
    Serial.print("Dist: ");
    Serial.print(distance);
    Serial.print(" Pitch: ");
    Serial.print(pitch);
    Serial.print(" Yaw: ");
    Serial.print(yaw);
    Serial.println("");
    lastPrintTime = currentMillis;
  }
  
}


