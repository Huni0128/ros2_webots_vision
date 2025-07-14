cmake_minimum_required(VERSION 3.8)
project(panda_description)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

################################################################################
# Install URDF / XACRO / meshes
################################################################################

# urdf 폴더 전체를 share/<pkg>/urdf 로 설치
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
    PATTERN "*.xacro"
    PATTERN "*.urdf"
    PATTERN "*.yaml"
    PATTERN "*.dae"
    PATTERN "*.stl"
)

################################################################################
# ament package hooks
################################################################################

ament_package()
