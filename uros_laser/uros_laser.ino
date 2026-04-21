#include "ros.h"
#include "laser.h"
#include "imu.h"
#include <micro_ros_platformio.h>

#define BUTTON_PIN 15

// #define USE_ROS

const unsigned long imuInterval = 10;    // 100Hz for smooth control
const unsigned long laserInterval = 50;  // 20Hz is usually plenty for distance
const unsigned long printInterval = 100; // 10Hz for the Serial Monitor

unsigned long lastImuTime = 0;
unsigned long lastLaserTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  Serial.begin(115200);
  #ifdef USE_ROS
    setupMicroROS();
  #endif

  setupIMU();
  setupDistance();
  
  

  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // change to external resistor later for more battery-life
  delay(2000);

}

void loop() {
  unsigned long currentMillis = millis();
  updateIMU();
  if (currentMillis - lastLaserTime >= laserInterval) {
    updateDistance();
    lastLaserTime = currentMillis;
  }
  

  if (digitalRead(BUTTON_PIN) == LOW) { // publish data while button is held down
      #ifndef USE_ROS
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

      #else

      // imu_msg.orientation.x = qx;
      // imu_msg.orientation.y = qy;
      // imu_msg.orientation.z = qz;
      // imu_msg.orientation.w = qw;

      // imu_msg.linear_acceleration.x = acc_x;
      // imu_msg.linear_acceleration.y = acc_y;
      // imu_msg.linear_acceleration.z = acc_z;

      distance_msg.data = distance;

      // RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
      RCSOFTCHECK(rcl_publish(&distance_publisher, &distance_msg, NULL));

      #endif
  }
  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100))); // Not needed for publisher-only node
}
