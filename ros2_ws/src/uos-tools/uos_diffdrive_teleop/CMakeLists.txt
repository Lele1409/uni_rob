# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.8)

project(uos_diffdrive_teleop)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_executable(uos_diffdrive_teleop_key src/uos_diffdrive_teleop.cpp src/uos_diffdrive_teleop_key.cpp)
ament_target_dependencies(uos_diffdrive_teleop_key geometry_msgs sensor_msgs rclcpp)

install(TARGETS
    uos_diffdrive_teleop_key
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch/
    DESTINATION DESTINATION share/${PROJECT_NAME}
)

ament_package()