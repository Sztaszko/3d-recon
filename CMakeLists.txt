cmake_minimum_required(VERSION 3.5)

project(3d-recon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Ceres  REQUIRED)
find_package(Boost  REQUIRED program_options)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})

include_directories(3d-recon PRIVATE inc/ src/)

add_executable(3d-recon main.cpp inc/reconstruction.h
    inc/cameracalibrator.h
    inc/camera.h
    inc/settings.h
    inc/safequeue.h
    src/reconstruction.cpp
    src/cameracalibrator.cpp
    src/camera.cpp)


set(3d-recon-LIBS
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
    )

target_link_libraries(3d-recon ${3d-recon-LIBS})
