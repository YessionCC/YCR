
cmake_minimum_required(VERSION 3.5)

project(ycr CXX CUDA)

set(CMAKE_CXX_FLAGS "-g -Werror=return-type -Wall -std=c++11") 
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV 4.0)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIR})
aux_source_directory(${PROJECT_SOURCE_DIR}/src CPP_SRCS)  

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(EXE_FILE test3)
add_library(ycr SHARED ${CPP_SRCS})
add_executable(${EXE_FILE} ${PROJECT_SOURCE_DIR}/tests/${EXE_FILE}.cpp)

target_link_libraries(${EXE_FILE} 
  ycr assimp yaml-cpp pthread ${OpenCV_LIBS}
)