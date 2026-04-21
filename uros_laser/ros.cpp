#include "ros.h"

rcl_publisher_t imu_publisher;
rcl_publisher_t distance_publisher;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__Float32 distance_msg;


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setupMicroROS() {
  set_microros_serial_transports(Serial);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "laser_node", "", &support));

  //init messsage default values
  // sensor_msgs__msg__Imu__init(&imu_msg);
  std_msgs__msg__Float32__init(&distance_msg);

  // Create publishers
  // RCCHECK(rclc_publisher_init_default(
  //   &imu_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //   "laser/imu/data"));

  RCCHECK(rclc_publisher_init_default(
    &distance_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "laser/distance/data"));

  
}