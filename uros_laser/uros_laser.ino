#include "laser.h"
#include "imu.h"
# include <Eigen.h>
#include<Eigen/Dense>

#define LED_BUILTIN 2




void setup() {
  Serial.begin(115200);
  setupDistance();
  setupIMU();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  

  delay(2000);
}



Eigen::Matrix4d transformationTarget = Matrix4d::Identity();
Eigen::Matrix4d transformationRobot = Matrix4d::Identity();

void loop() {
  updateDistance();
  updateIMU();


  Eigen::Matrix3d Rz;
  Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw),  cos(yaw), 0,
        0,   0,  1;

  Eigen::Matrix3d Ry;
  Ry << cos(pitch), 0, sin(pitch),
        0,  1, 0,
        -sin(pitch), 0, cos(pitch);

  Eigen::Matrix3d R = Rz*Ry;
  Eigen::Vector3d Point(distance, 0, 0);

  Eigen::Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0,0) = R;
  T.block<3, 1>(0,3) = Point;


  Serial.print("Distance: ");
  Serial.println(distance);

  Serial.print("Pitch: ");
  Serial.println(pitch);

  Serial.print("Yaw: ");
  Serial.println(yaw);

  delay(100);
}
