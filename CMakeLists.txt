# cmake file for ulsBankSleepDetection project
# author : lampson.song
# date   : 2018-2-28

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

project(ulsBankSleep)

set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O3 -Wall")
add_definitions(-std=c++14)

set(PROJECT_BINARY_DIR build)
set(LIBRARY_OUTPUT_PATH lib)

## custome settings ##

set(OpenCV_DIR /usr/local/opencv32/share/OpenCV)
set(FFMPEG_DIR /usr/local/nvencFFMPEG)
set(CUDA_DIR /usr/local/cuda)

## custome settings end ##

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

set(CAFFE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/caffe/distribute)

# 50 53 60 61
set(GENCODE -gencode arch=compute_61,code=sm_61)
list(APPEND CUDA_NVCC_FLAGS "-std=c++11" ${GENCODE})

include_directories(src 
  ${CAFFE_DIR}/include 
  ${OpenCV_DIR} 
  ${CUDA_DIR}/include
  ${FFMPEG_DIR}/include 
  /usr/lib/x86_64-linux-gnu)

link_directories(${CUDA_DIR}/lib 
  ${FFMPEG_DIR}/lib 
  ${CAFFE_DIR}/lib)

add_subdirectory(src)
