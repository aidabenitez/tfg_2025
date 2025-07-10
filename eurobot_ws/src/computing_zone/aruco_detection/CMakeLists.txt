cmake_minimum_required(VERSION 3.1)
project(aruco_detection)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

include_directories(${OPENCV_INCLUDE_DIRS})

set(exec
    src/pose_estimation.cpp
)

add_executable(t_aruco_detection ${exec})

target_link_libraries(t_aruco_detection
    ${OpenCV_LIBRARIES}
)

target_compile_options(t_aruco_detection
    PRIVATE -O3
)
