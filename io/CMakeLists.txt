project(io)
cmake_minimum_required(VERSION 2.8.3)

find_package(Eigen3)
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(yamltest yamltest.cc)
target_link_libraries(yamltest yaml-cpp)