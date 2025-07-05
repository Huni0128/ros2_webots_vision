cmake_minimum_required(VERSION 3.16)
project(extern_controller)

# Webots 설치 경로 환경변수 사용
set(WEBOTS_HOME $ENV{WEBOTS_HOME})

# Include 헤더 디렉토리
include_directories(${WEBOTS_HOME}/include/controller/cpp)

# 링크할 라이브러리 디렉토리
link_directories(${WEBOTS_HOME}/lib/controller)

# 실행 파일 생성
add_executable(extern extern.cpp)

# Webots C++ 컨트롤러 라이브러리 링크 (올바른 이름: CppController)
target_link_libraries(extern CppController)
