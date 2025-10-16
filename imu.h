#pragma once

extern float pitch;
extern float yaw;

extern double Ry[3][3];
extern double Rz[3][3];

void setupIMU();
void updateIMU();