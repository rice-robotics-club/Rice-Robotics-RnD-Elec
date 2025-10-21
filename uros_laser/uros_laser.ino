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


Matrix4d transformationMatrix(){
  updateDistance();
  updateIMU();

  double cyaw = cos(yaw * M_PI / 180.0);
  double syaw = sin(yaw * M_PI / 180.0);
  double cpitch = cos(pitch * M_PI / 180.0);
  double spitch = sin(pitch * M_PI / 180.0);

  Eigen::Matrix3d Rz;
  Rz << cyaw, -syaw, 0,
        syaw,  cyaw, 0,
        0,   0,  1;

  Eigen::Matrix3d Ry;
  Ry << cpitch, 0, spitch,
        0,  1, 0,
        -spitch, 0, cpitch;

  Eigen::Matrix3d R = Rz*Ry;
  Eigen::Vector3d Point(distance, 0, 0);

  Eigen::Matrix4d T = Matrix4d::Identity();
  T.block<3, 3>(0,0) = R;
  T.block<3, 1>(0,3) = Point;
  
  return T;
}


void loop() {
  Matrix4d T = transformationMatrix();
  
  Serial.print("Distance: ");
  Serial.println(distance);

  Serial.print("Pitch: ");
  Serial.println(pitch);

  Serial.print("Yaw: ");
  Serial.println(yaw);

  delay(100);
}

