cmake_minimum_required(VERSION 3.8)
project(go2w_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY
    launch
    src
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()
