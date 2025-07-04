cmake_minimum_required(VERSION 3.16)
project(panda_gui)

set(CMAKE_CXX_STANDARD 17)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(nlohmann_json REQUIRED)

add_executable(panda_gui panda_gui.cpp)

target_link_libraries(panda_gui
  Qt6::Widgets
  nlohmann_json::nlohmann_json
)
