cmake_minimum_required(VERSION 3.5)
project(rdj2025_rqt_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)

install(
  PROGRAMS
    rdj2025_rqt_camera/camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
