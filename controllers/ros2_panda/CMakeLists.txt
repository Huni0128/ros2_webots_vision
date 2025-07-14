cmake_minimum_required(VERSION 3.10)
project(ros2_panda)

set(CMAKE_CXX_STANDARD 17)

# --------------------------------------------
# Webots 헤더 경로와 라이브러리 설정
# --------------------------------------------
set(WEBOTS_INCLUDE_DIR /usr/local/webots/include/controller)
set(WEBOTS_LIBRARY     /usr/local/webots/lib/controller/libCppController.so)

# --------------------------------------------
# 필요한 ROS 2 패키지 찾기
# --------------------------------------------
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros       REQUIRED)

# --------------------------------------------
# 노드 실행 파일 생성 및 소스 파일 지정
# --------------------------------------------
add_executable(ros2_panda
  src/ros2_panda.cpp
  src/camera_publisher.cpp
  src/panda_joint_controller.cpp
  src/panda_gripper_controller.cpp
  src/panda_tf_publisher.cpp
)

# --------------------------------------------
# 포함 디렉터리 설정 (Webots 헤더와 패키지 헤더)
# --------------------------------------------
target_include_directories(ros2_panda PRIVATE
  ${WEBOTS_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# --------------------------------------------
# ROS 2 의존성 연결
# --------------------------------------------
ament_target_dependencies(ros2_panda
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# --------------------------------------------
# Webots 라이브러리 및 시스템 라이브러리 연결
# --------------------------------------------
target_link_libraries(ros2_panda
  ${WEBOTS_LIBRARY}
  dl
)

# --------------------------------------------
# 실행 파일 설치 (install/lib/ros2_panda/)
# --------------------------------------------
install(TARGETS ros2_panda
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------
# Launch 파일 설치 (install/share/ros2_panda/launch/)
# --------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------
# ament 패키지 설정
# --------------------------------------------
ament_package()
