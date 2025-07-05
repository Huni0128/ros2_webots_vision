cmake_minimum_required(VERSION 3.10)
project(ros2_panda)

set(CMAKE_CXX_STANDARD 17)

# Webots 헤더 및 라이브러리
set(WEBOTS_INCLUDE_DIR /usr/local/webots/include/controller)
set(WEBOTS_LIBRARY /usr/local/webots/lib/controller/libCppController.so)

# ROS 2 패키지들
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 실행 파일
add_executable(ros2_panda
  src/ros2_panda.cpp
  src/camera_publisher.cpp
  src/panda_joint_controller.cpp
  src/panda_gripper_controller.cpp
)

target_include_directories(ros2_panda PRIVATE
  ${WEBOTS_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(ros2_panda
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(ros2_panda
  ${WEBOTS_LIBRARY}
  dl
)

add_custom_command(TARGET ros2_panda POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ros2_panda> ${CMAKE_CURRENT_SOURCE_DIR}/ros2_panda
)

ament_package()
