cmake_minimum_required(VERSION 3.1)

project(cnc-vision LANGUAGES CXX)

macro(add_deployment_file SRC DEST)
    file(RELATIVE_PATH path ${CMAKE_SOURCE_DIR}
         ${CMAKE_CURRENT_SOURCE_DIR})
    file(APPEND "${CMAKE_BINARY_DIR}/QtCreatorDeployment.txt"
         "${path}/${SRC}:${DEST}\n")
endmacro()

option(CNCVISION "CNC vision board")
option(DESKTOP "Desktop - all in one")
option(ANDROIDGUI "Android GUI")


set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}")

if(CNCVISION)
    set(CMAKE_SYSROOT /home/woodenprint/tinker/sysroot)
    set(OpenCV_DIR /home/woodenprint/tinker/sysroot/usr/lib/arm-linux-gnueabihf/cmake/opencv4/)
    link_directories(/home/woodenprint/tinker/sysroot/usr/local/lib)

    set(CMAKE_INSTALL_PREFIX "/home/linaro/cnc-vision")

    file(WRITE "${CMAKE_BINARY_DIR}/QtCreatorDeployment.txt"
               "${CMAKE_INSTALL_PREFIX}\n")
    #file(RELATIVE_PATH relative_exe_path
    #     "${CMAKE_CURRENT_SOURCE_DIR}"
    #     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}")
    #add_deployment_file(relative_exe_path bin)
endif()
if(DESKTOP)
    SET(GCC_COVERAGE_COMPILE_FLAGS "-lstdc++")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
    #set(OpenCV_DIR /usr/lib/python3/dist-packages)
    link_directories(/usr/local/lib/)
endif()
if(ANDROIDGUI)

endif()

find_package(Qt5 COMPONENTS Core Quick Qml Multimedia Charts REQUIRED)
find_package(OpenCV REQUIRED)

#message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

file(GLOB_RECURSE sources *.cpp *.h)

add_executable(${PROJECT_NAME}
        ${sources}
        "quick.qrc"
        )

include_directories(${OpenCV_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWidhDebInfo>>:QY_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBS} PRIVATE Qt5::Core Qt5::Quick Qt5::Qml Qt5::Multimedia Qt5::Charts)
