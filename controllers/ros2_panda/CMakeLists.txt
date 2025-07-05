cmake_minimum_required(VERSION 3.10)
project(ros2_panda)

set(CMAKE_CXX_STANDARD 17)

# Webots 헤더 및 라이브러리
set(WEBOTS_INCLUDE_DIR /usr/local/webots/include/controller)
set(WEBOTS_LIBRARY /usr/local/webots/lib/controller/libCppController.so)

# ROS 2 패키지들
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ros2_panda
  src/ros2_panda.cpp
  src/camera_publisher.cpp
)

target_include_directories(ros2_panda PRIVATE
  ${WEBOTS_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(ros2_panda
  rclcpp
  sensor_msgs
)

target_link_libraries(ros2_panda
  ${WEBOTS_LIBRARY}
  dl
)

install(TARGETS ros2_panda
        RUNTIME DESTINATION ${CMAKE_SOURCE_DIR})

ament_package()
