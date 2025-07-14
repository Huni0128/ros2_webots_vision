cmake_minimum_required(VERSION 3.10)
project(ros2_panda)

set(CMAKE_CXX_STANDARD 17)

# Webots 헤더 경로 설정
set(WEBOTS_INCLUDE_DIR /usr/local/webots/include/controller)

# Webots 라이브러리 경로 설정
set(WEBOTS_LIBRARY     /usr/local/webots/lib/controller/libCppController.so)

# ROS 2 기본 패키지 찾기
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros       REQUIRED)

# 실행 파일 빌드 대상 정의
add_executable(ros2_panda
  src/ros2_panda.cpp
  src/camera_publisher.cpp
  src/panda_joint_controller.cpp
  src/panda_gripper_controller.cpp
  src/panda_tf_publisher.cpp
)

# include 디렉터리 설정
target_include_directories(ros2_panda PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${WEBOTS_INCLUDE_DIR}
)

# ROS 2 의존성 연결
ament_target_dependencies(ros2_panda
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# Webots 및 시스템 라이브러리 연결
target_link_libraries(ros2_panda
  ${WEBOTS_LIBRARY}
  dl
)

# 빌드 후 Webots controllers 폴더로 실행파일 복사
add_custom_command(
  TARGET ros2_panda
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:ros2_panda>
    ${CMAKE_CURRENT_SOURCE_DIR}/ros2_panda
)

# 실행 파일 설치 설정
install(TARGETS ros2_panda
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일 설치 설정
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
