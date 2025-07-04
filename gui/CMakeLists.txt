# CMake 최소 요구 버전 설정
cmake_minimum_required(VERSION 3.16)

# 프로젝트 이름 설정
project(panda_gui)

# C++17 사용 설정
set(CMAKE_CXX_STANDARD 17)

# Qt6의 Widgets 모듈 찾기
find_package(Qt6 REQUIRED COMPONENTS Widgets)

# nlohmann_json 패키지 찾기 (https://github.com/nlohmann/json)
find_package(nlohmann_json REQUIRED)

# 실행 파일 생성: panda_gui.cpp → panda_gui
add_executable(panda_gui panda_gui.cpp)

# 라이브러리 연결: Qt6::Widgets 및 nlohmann_json
target_link_libraries(panda_gui
  Qt6::Widgets
  nlohmann_json::nlohmann_json
)
