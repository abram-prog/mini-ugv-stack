cmake_minimum_required(VERSION 3.8)
project(sensor_driver_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_publisher src/imu_publisher.cpp)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs)

install(TARGETS imu_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
