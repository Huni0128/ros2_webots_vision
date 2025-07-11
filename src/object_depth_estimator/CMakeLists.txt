cmake_minimum_required(VERSION 3.8)
project(object_depth_estimator)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존 패키지 찾기
find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(vision_msgs       REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(cv_bridge         REQUIRED)
find_package(OpenCV            REQUIRED)
find_package(message_filters   REQUIRED)

# 실행 파일 선언
add_executable(object_depth_estimator
    src/object_depth_estimator.cpp
)

# ROS 2 의존성 연결
ament_target_dependencies(object_depth_estimator
    rclcpp
    vision_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    OpenCV
    message_filters
)

# 설치 설정
install(TARGETS object_depth_estimator
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
