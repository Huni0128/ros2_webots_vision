TARGET = panda_controller
CXX_SOURCES = panda_controller.cpp

# OpenCV 경로 추가 (opencv4 기준)
INCLUDE = $(shell pkg-config --cflags opencv4)
LIBRARIES = $(shell pkg-config --libs opencv4)

# Webots 기본 Makefile 포함
include $(WEBOTS_HOME)/resources/Makefile.include

# OpenCV 포함시키도록 컴파일러 옵션 확장
CXXFLAGS += $(INCLUDE)
LIBS += $(LIBRARIES)
