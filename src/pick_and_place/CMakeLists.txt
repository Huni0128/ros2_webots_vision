cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

# 컴파일 옵션 설정 (경고 표시)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament 필수
find_package(ament_cmake REQUIRED)

# ROS 2 의존성 패키지
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(vision_msgs   REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2           REQUIRED)
find_package(tf2_ros       REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# include 디렉토리 추가
include_directories(include)

# command_sub 노드 빌드
add_executable(command_sub src/command_sub.cpp)
ament_target_dependencies(command_sub
    rclcpp
    std_msgs
    vision_msgs
)

# scan_move 노드 빌드
add_executable(scan_move src/scan_move.cpp)
ament_target_dependencies(scan_move
    rclcpp
    std_msgs
    sensor_msgs
    vision_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# center_control 노드 빌드
add_executable(center_control src/center_control.cpp)
ament_target_dependencies(center_control
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
)

# 실행 파일 설치
install(
    TARGETS
        command_sub
        scan_move
        center_control
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# launch 디렉토리 설치
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# include 디렉토리 설치
install(
    DIRECTORY include/
    DESTINATION include/
)

ament_package()
