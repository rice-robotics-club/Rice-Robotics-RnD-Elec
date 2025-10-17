#include "ros.h"

rcl_publisher_t publisher;
geometry_msgs__msg__Vector3 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void setupMicroROS() {
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();

  // Initialize support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "laser_pointer_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "laser_point_readings"));

  msg.x = 0; //distance
  msg.y = 0; //pitch
  msg.z = 0; //yaw 
}