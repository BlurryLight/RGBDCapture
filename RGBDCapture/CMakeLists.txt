cmake_minimum_required(VERSION 2.6.0)
project(RGBDCapture)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}")
set(CMAKE_CXX_STANDARD 11)

find_package(ZLIB REQUIRED)
set(OpenCV_DIR /home/panda/opt/opencv412-build/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)
if(OpenCV_VERSION VERSION_LESS 3.0.0)
    message(FATAL_ERROR "OPENCV needs to be 3.x/4.x!")
endif(OpenCV_VERSION VERSION_LESS 3.0.0)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)


set(OPENNI_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../OpenNI2/Include/")
set(OPENNI_BIN_DIR "${CMAKE_SOURCE_DIR}/../OpenNI2/Bin/x64-Release/")
message(${OPENNI_BIN_DIR})
INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR})
message(${OpenCV_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${OPENNI_BIN_DIR})

aux_source_directory(. DIR_SRCS)

option(REGISTRATION "REGISTRATION DEPTH TO COLOR" ON)
option(IMSHOW "CV::IMSHOW" ON)
option(SAVEJPG "RGB images will be saved as compressed jpg" OFF)
option(SAVETIMESTAMP "Images will be named for timestamps otherwise index" OFF)

if(NOT IMSHOW)
    add_definitions(-DCV_NOSHOW)
endif(NOT IMSHOW)

if(SAVEJPG)
    add_definitions(-DSAVEJPG)
endif(SAVEJPG)
if(SAVETIMESTAMP)
    add_definitions(-DSAVETIMESTAMP)
endif(SAVETIMESTAMP)

add_executable(RGBDCapture ${DIR_SRCS})

target_link_libraries(RGBDCapture
                      ${ZLIB_LIBRARY}
                      ${OpenNI_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${Boost_SYSTEM_LIBRARIES}
                      ${Boost_FILESYSTEM_LIBRARIES}
                      OpenNI2
                      )

if(REGISTRATION)
    set_target_properties(RGBDCapture PROPERTIES OUTPUT_NAME "RGBDCapture_rgs")
    target_compile_definitions(RGBDCapture PRIVATE REGISTRATION)
endif(REGISTRATION)


