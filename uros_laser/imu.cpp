#include <Arduino.h>
#include <cmath>

#define ICM_20948_USE_DMP
#include "ICM_20948.h"

#define WIRE_PORT Wire
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM;
#else
ICM_20948_I2C myICM;
#endif

#define SERIAL_PORT Serial

double qx, qy, qz, qw;
float acc_x, acc_y, acc_z;
float pitch = 0, roll = 0, yaw = 0;

void setupIMU() {
  WIRE_PORT.begin(21, 22);
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  bool success = true;

  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable sensors
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);

  // ODR configuration (match stable example)
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);       // ~5 Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);       // ~1 Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);        // ~1 Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok); // ~1 Hz

  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!")); 
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    while (1);
  }
}

void updateIMU() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || 
      (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    if (data.header & DMP_header_bitmap_Quat6)
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

      double sum = (q1*q1 + q2*q2 + q3*q3);
      double q0 = sqrt(sum > 1.0 ? 0.0 : 1.0 - sum);

      qw = q0;
      qx = q2;
      qy = q1;
      qz = -q3;

      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      roll = atan2(t0, t1) * 180.0 / PI;

      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(t3, t4) * 180.0 / PI;
    }

    if (data.header & DMP_header_bitmap_Accel)
    {
      acc_x = (float)data.Raw_Accel.Data.X;
      acc_y = (float)data.Raw_Accel.Data.Y;
      acc_z = (float)data.Raw_Accel.Data.Z;
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)
  {
    delay(10);
  }
}