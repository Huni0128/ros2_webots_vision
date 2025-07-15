cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

# 컴파일 옵션 설정 (GCC 또는 Clang 사용 시)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# 테스트 관련 설정 (선택사항)
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
)

# 실행 파일 설치 경로 설정
install(
  TARGETS command_sub
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ament 패키지로 등록
ament_package()
