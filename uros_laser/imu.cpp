#include "imu.h"
#include "ICM_20948.h"

ICM_20948_I2C myICM;
float roll, pitch, yaw;

void setupIMU() {
    Wire.begin();
    Wire.setClock(400000);

    bool initialized = false;
    while (!initialized) {
        myICM.begin(Wire, 1); 
        if (myICM.status != ICM_20948_Stat_Ok) {
            delay(500);
        } else {
            initialized = true;
        }
    }

    myICM.initializeDMP();
    myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
    myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0); 
    myICM.enableFIFO();
    myICM.enableDMP();
    myICM.resetDMP();
    myICM.resetFIFO();
}

void updateIMU() {
    icm_20948_DMP_data_t data;
    
    do {
        myICM.readDMPdataFromFIFO(&data);
        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
            if ((data.header & DMP_header_bitmap_Quat6) > 0) {
                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                // Coordinate transformation for Euler Angles
                double qw = q0;
                double qx = q2;
                double qy = q1;
                double qz = -q3;

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
        }
    } while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail);
}
