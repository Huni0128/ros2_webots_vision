cmake_minimum_required(VERSION 3.16)
project(vision_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joint_commander src/vision_sim/controller_node.cpp)
ament_target_dependencies(joint_commander rclcpp std_msgs)

install(TARGETS joint_commander
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY protos
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY controllers/extern
  DESTINATION share/${PROJECT_NAME}/controllers
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
