#include "laser.h"
#include "imu.h"

#define LED_BUILTIN 2

void setup() {
  Serial.begin(115200);
  setupDistance();
  setupIMU();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  delay(2000);
}


void loop() {
  updateDistance();
  updateIMU();

  Serial.print("Distance: ");
  Serial.println(distance);

  Serial.print("Pitch: ");
  Serial.println(pitch);

  Serial.print("Yaw: ");
  Serial.println(yaw);

  delay(100);
}
