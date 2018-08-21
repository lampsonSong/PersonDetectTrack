# src CMakeLists, compile the source files
# author : lampson.song
# Date   : 2018-2-28

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/)
    
file(GLOB_RECURSE CPP_SRCS *.cpp)
file(GLOB_RECURSE CPP_HEAD *.h *.hpp)

file(GLOB_RECURSE CU_SRCS *.cu)

CUDA_COMPILE(CU_OBJS ${CU_SRCS})
add_executable(ulsBankSleep ${CPP_SRCS} ${CPP_HEAD} ${CU_SRCS})

target_link_libraries(ulsBankSleep ${CU_OBJS} ${OpenCV_LIBS} caffe cudart boost_system boost_filesystem pthread glog gflags avcodec avutil)
