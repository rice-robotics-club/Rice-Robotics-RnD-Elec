#include "ros.h"
#include "laser.h"
#include "imu.h"
#include <micro_ros_platformio.h>

#define BUTTON_PIN 15

void setup() {
  setupMicroROS();

  setupDistance();
  setupIMU();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // change to external resistor later for more battery-life
  delay(2000);

}

void loop() {
  updateDistance();
  updateIMU();

  if (digitalRead(BUTTON_PIN) == LOW) { // publish data while button is held down
      imu_msg.orientation.x = qx;
      imu_msg.orientation.y = qy;
      imu_msg.orientation.z = qz;
      imu_msg.orientation.w = qw;

      imu_msg.linear_acceleration.x = acc_x;
      imu_msg.linear_acceleration.y = acc_y;
      imu_msg.linear_acceleration.z = acc_z;

      distance_msg.data = distance;

      rcl_publish(&imu_publisher, &imu_msg, NULL);
      rcl_publish(&distance_publisher, &distance_msg, NULL);
  }
  delay(100);
  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Not needed for publisher-only node
}
