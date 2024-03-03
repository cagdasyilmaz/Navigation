cmake_minimum_required(VERSION 3.25.2)
project(Navigation)

set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_CXX_COMPILER "/usr/bin/g++-11")
#set(CMAKE_CXX_COMPILER "/usr/bin/clang++-12")
set(CMAKE_CXX_COMPILER "/opt/intel/oneapi/compiler/2023.1.0/linux/bin/icpx")

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math")

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Eigen3 REQUIRED)
include_directories("${EIGEN3_INCLUDE_DIR}")

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(Navigation ${SOURCES})
target_link_libraries(Navigation Eigen3::Eigen)
