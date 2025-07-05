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

# 실행 파일: ros2_panda (카메라 + 관절 제어 통합)
add_executable(ros2_panda
  src/ros2_panda.cpp
  src/camera_publisher.cpp
  src/panda_joint_controller.cpp  # <-- 여기 포함 (별도 실행 파일 제거)
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

# 설치
install(TARGETS
  ros2_panda
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
