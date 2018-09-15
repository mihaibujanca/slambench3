cmake_minimum_required(VERSION 2.8)

project(slambench2_repository)

SET(CUDA_HOST_COMPILER "/usr/bin/gcc-7")


# CMake variables
#######################################################

MESSAGE(STATUS CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH})

SET(CMAKE_MODULE_PATH /home/paul/git/slambench2/cmake_modules)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin CACHE PATH "Build directory" FORCE)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib CACHE PATH "Build directory" FORCE)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET(CMAKE_BUILD_FILES_DIRECTORY ${PROJECT_BINARY_DIR}/bfd)
SET(CMAKE_BINARY_DIR  ${PROJECT_BINARY_DIR}/bindir)
SET(CMAKE_CACHEFILE_DIR  ${PROJECT_BINARY_DIR}/cache)
SET(CMAKE_BUILD_DIRECTORY ${PROJECT_BINARY_DIR}/builddir)




add_subdirectory(framework)  # SLAMBench 

add_subdirectory(benchmarks) # Benchmarks
