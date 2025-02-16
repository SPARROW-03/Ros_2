cmake_minimum_required(VERSION 3.5)
project(micro_ros_led_blinker)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add the executable for LED blinker node
add_executable(led_blinker src/led_blinker.cpp)

# Link the necessary dependencies
ament_target_dependencies(led_blinker rclcpp std_msgs visualization_msgs)

# Install the node
install(TARGETS
  led_blinker
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
