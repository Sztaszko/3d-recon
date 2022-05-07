cmake_minimum_required(VERSION 3.5)

project(3d-recon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Ceres  REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

add_executable(3d-recon main.cpp)

target_link_libraries(3d-recon ${OpenCV_LIBS})