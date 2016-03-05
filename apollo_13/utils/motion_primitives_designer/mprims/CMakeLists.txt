cmake_minimum_required(VERSION 2.8)
project(mprims)

find_package(Qt4 REQUIRED COMPONENTS QtOpenGL QtGui QtCore)
find_package(OpenGL REQUIRED)

include(${QT_USE_FILE})
include_directories("/usr/include/eigen3")
include_directories(${OPENGL_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(src)
