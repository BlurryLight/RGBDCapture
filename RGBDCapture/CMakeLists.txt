cmake_minimum_required(VERSION 2.6.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}")

find_package(ZLIB REQUIRED)
set(OPENCV_DIR /home/panda/opt/opencv-2.4.13.6/release)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)


set(OPENNI_INCLUDE_DIR "/usr/include/openni2/")
INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})

aux_source_directory(. DIR_SRCS)

option(REGISTRATION "REGISTRATION DEPTH TO COLOR" ON)

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

# use C++11
target_compile_features(RGBDCapture PRIVATE cxx_range_for)


