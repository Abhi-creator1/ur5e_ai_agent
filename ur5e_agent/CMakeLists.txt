cmake_minimum_required(VERSION 3.8)
project(ur5e_agent)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# Install Python modules
#ament_python_install_module(${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts and tools
install(
  DIRECTORY 
    scripts/
    scripts/tools/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)
ament_package()
