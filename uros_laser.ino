#include "ros.h"
#include "laser.h"
#include "imu.h"
#include <micro_ros_arduino.h>

enum scanState {
  TARGET = 0,
  ROBOT = 1,
  NONE = 2,
}

bool buttonDown = false;

void setup() {
  scanState currentState = NONE;
  setupMicroROS();
  setupDistance();
  setupIMU();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(2000);

  delay(2000);
}

void nextState() {
  currentState = (currentState + 1) % 3
}

double[3][3] getReadings() {
  updateDistance();
  updateIMU();

  return Rz@Ry;
}

void loop() {
  if (buttonDown) {
    if (currentState == TARGET) {
      double transformationTarget[3][3] = Rz@Ry;
      double col1[3];
      for (int i = 0; i<3; i++{
         col1[i]= transformtionTarget[i][0]
      })
      double newColumn[3] = distance*col1
      for (int )

      for (int i = 0; i < 3; i++) {
        transformationTarget[i][3] = newColumn[i];
      }

    } else if (currentState == ROBOT) {
      
    } else {

    }

  }

  

  rcl_publish(&publisher, &msg, NULL);
  delay(100);
  //RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
