#pragma once
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/vector3.h>

// Globals
extern rcl_publisher_t publisher;
extern geometry_msgs__msg__Vector3 msg;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

// Macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define LED_BUILTIN 2

// Function declarations
void error_loop();
void setupMicroROS();   // encapsulate ROS init logic