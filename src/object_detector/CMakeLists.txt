cmake_minimum_required(VERSION 3.5)
project(object_detector)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python 패키지 설치
ament_python_install_package(${PROJECT_NAME})

# package.xml 복사
install(
    FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

# 실행 스크립트 설치
install(
    PROGRAMS
        scripts/object_detector
    DESTINATION lib/${PROJECT_NAME}
)

# 모델 디렉터리 설치
install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
