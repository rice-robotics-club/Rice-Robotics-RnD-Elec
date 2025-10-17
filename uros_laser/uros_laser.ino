#include "ros.h"
#include "laser.h"
#include "imu.h"
#include <micro_ros_arduino.h>

void setup() {
  setupMicroROS();
  setupDistance();
  setupIMU();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(2000);

  delay(2000);
}

void loop() {
  updateDistance();
  updateIMU();

  rcl_publish(&publisher, &msg, NULL);
  delay(100);
  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
