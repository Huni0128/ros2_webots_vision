cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

# 컴파일 옵션 (GCC/Clang 공통)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament
find_package(ament_cmake REQUIRED)

# ROS 2 기본 의존성
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # scan_move에서 JointState 사용

# include 디렉토리 인클루드
include_directories(
  include
)

# ─── command_sub 노드 ───────────────────────────────────────────────
add_executable(command_sub src/command_sub.cpp)
ament_target_dependencies(command_sub
  rclcpp
  std_msgs
  vision_msgs
)

# ─── scan_move 노드 ─────────────────────────────────────────────────
add_executable(scan_move src/scan_move.cpp)
ament_target_dependencies(scan_move
  rclcpp
  std_msgs
  sensor_msgs
)

# ─── 설치 설정 ───────────────────────────────────────────────────────
install(
  TARGETS
    command_sub
    scan_move
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# launch 파일 설치
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 헤더가 있으면 include/ 아래 전체 설치
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
