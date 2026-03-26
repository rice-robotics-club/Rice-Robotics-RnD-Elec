
#include <Wire.h>
#include "imu.h"
#include "laser.h"

#define SERIAL_PORT Serial

void setup() {
    SERIAL_PORT.begin(115200);
    setupIMU();
    setupDistance();
    
    delay(500); 
}

void loop() {
    updateIMU();
    updateDistance();
    
    SERIAL_PORT.print(distance, 3);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(roll, 2);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(pitch, 2);
    SERIAL_PORT.print(",");
    SERIAL_PORT.println(yaw, 2);
    delay(20); 
}

