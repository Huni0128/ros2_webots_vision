cmake_minimum_required(VERSION 3.5)
project(pyqt_gui)

find_package(ament_cmake REQUIRED)

# 1) Python 패키지 설치 및 marker 생성
ament_python_install_package(${PROJECT_NAME})

# 2) package.xml 설치
install(
    FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

# 3) 실행 가능한 스크립트 등록
install(
    PROGRAMS
        scripts/gui_node
    DESTINATION lib/${PROJECT_NAME}
)

# 4) 메타데이터 생성
ament_package()
