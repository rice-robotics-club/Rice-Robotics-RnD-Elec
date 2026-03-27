#pragma once

extern double qx, qy, qz, qw;
extern float acc_x, acc_y, acc_z;
extern float pitch;
extern float yaw;

void setupIMU();
void updateIMU();