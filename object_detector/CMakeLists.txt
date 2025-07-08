cmake_minimum_required(VERSION 3.5)
project(object_detector)

find_package(ament_cmake REQUIRED)

# 1) Python 패키지(object_detector/) 설치
ament_python_install_package(${PROJECT_NAME})

# 2) package.xml 을 share/… 아래로 복사
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# 3) 스크립트 실행 파일 등록
install(
  PROGRAMS
    scripts/object_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
