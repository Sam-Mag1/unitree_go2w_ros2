cmake_minimum_required(VERSION 3.8)
project(go2w_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add executable
add_executable(joint_publisher src/joint_publisher.cpp)
ament_target_dependencies(joint_publisher rclcpp trajectory_msgs)

# Install the resource directories
install(TARGETS
          joint_publisher
        DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
  DESTINATION share/${PROJECT_NAME}
)

# ament package macro
ament_package()
