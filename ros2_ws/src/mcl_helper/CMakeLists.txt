cmake_minimum_required(VERSION 3.8)
project(mcl_helper)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package (Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} )

include_directories(
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_library(mcl_helper src/util.cpp src/scan_simulator.cpp)
ament_target_dependencies(mcl_helper rclcpp std_msgs geometry_msgs nav_msgs tf2_geometry_msgs sensor_msgs tf2_ros visualization_msgs tf2_sensor_msgs)

add_executable(mcl_helper_test src/util.cpp src/scan_simulator_test.cpp)
target_link_libraries(mcl_helper_test mcl_helper)
ament_target_dependencies(mcl_helper_test rclcpp std_msgs geometry_msgs nav_msgs tf2_geometry_msgs sensor_msgs tf2_ros visualization_msgs tf2_sensor_msgs)

install(TARGETS
  mcl_helper
  mcl_helper_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  maps/
  DESTINATION DESTINATION share/${PROJECT_NAME}
)


ament_package()
