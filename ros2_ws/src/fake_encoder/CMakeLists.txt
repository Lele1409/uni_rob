cmake_minimum_required(VERSION 3.8)
project(fake_encoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)

add_executable(fake_encoder src/FakeEncoder.cpp)
ament_target_dependencies(fake_encoder rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros)

# Install the launch directory and urdfs
install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
  fake_encoder
  DESTINATION lib/${PROJECT_NAME})

ament_package()
