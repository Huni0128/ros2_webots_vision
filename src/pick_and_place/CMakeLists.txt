cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

# 컴파일 옵션 설정 (GCC 또는 Clang 사용 시)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament 기능
find_package(ament_cmake REQUIRED)

# ROS 2 기본 패키지
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# vision_msgs (Detection2DArray) 추가
find_package(vision_msgs REQUIRED)

# 다른 의존 패키지 (필요시 유지)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# 테스트 관련 (선택사항)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# command_sub 노드 추가
add_executable(command_sub
  src/command_sub.cpp
)

# command_sub 노드에 필요한 의존성 링크
ament_target_dependencies(command_sub
  rclcpp
  std_msgs
  vision_msgs
)

# 실행 파일 설치
install(
  TARGETS command_sub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
