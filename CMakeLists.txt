cmake_minimum_required(VERSION 3.1)

project(cnc-vision LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
#set(CMAKE_SYSROOT /home/woodenprint/tinker/sysroot)

find_package(Qt5 COMPONENTS Core Quick Qml Multimedia Charts REQUIRED)
#find_package(ZeroMQ REQUIRED)
set(OpenCV_DIR /usr/lib/python3/dist-packages)
#set(OpenCV_DIR /home/woodenprint/tinker/sysroot/usr/lib/arm-linux-gnueabihf/cmake/opencv4/)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

#include_directories(/usr/local/include/SPLINTER/)
link_directories(/usr/local/lib/)
#link_directories(/home/woodenprint/tinker/sysroot/usr/local/lib)

file(GLOB_RECURSE sources *.cpp *.h)

add_executable(${PROJECT_NAME}
        ${sources}
        "quick.qrc"
        )

include_directories(${OpenCV_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWidhDebInfo>>:QY_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} PRIVATE Qt5::Core Qt5::Quick Qt5::Qml Qt5::Multimedia Qt5::Charts)
