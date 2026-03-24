#pragma once
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/quaternion.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>

extern sensor_msgs__msg__Imu imu_msg;
extern std_msgs__msg__Float32 distance_msg;

extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t distance_publisher;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define LED_BUILTIN 2

void error_loop();
void setupMicroROS(); 