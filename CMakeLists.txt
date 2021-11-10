
cmake_minimum_required(VERSION 3.5)

project(ycr CXX CUDA)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11") 
set(CMAKE_BUILD_TYPE "Release")

#aux_source_directory: not recursive
aux_source_directory(${PROJECT_SOURCE_DIR}/src CPP_SRCS)  
#aux_source_directory(${PROJECT_SOURCE_DIR}/src/gpu CUDA_SRCS)  

add_library(ycr SHARED ${CPP_SRCS})

#include_directories: is recursive
include_directories(${PROJECT_SOURCE_DIR}/include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

set(EXE_FILE test2)

add_executable(${EXE_FILE} ${PROJECT_SOURCE_DIR}/tests/${EXE_FILE}.cpp)
add_dependencies(${EXE_FILE} ycr)

target_link_libraries(${EXE_FILE} 
  ycr assimp
)