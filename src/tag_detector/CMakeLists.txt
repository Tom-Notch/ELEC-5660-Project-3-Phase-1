cmake_minimum_required(VERSION 2.8.3)
project(tag_detector)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-march=native -DEIGEN_DONT_PARALLELIZE")

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-Wno-deprecated)

# link libs only when needed
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge)

find_package(OpenCV REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(CMAKE_MODULE_PATH /usr/local/lib/cmake/)

# Use ArUco as a third-party library to be built with the project
add_subdirectory(${PROJECT_SOURCE_DIR}/third-party/aruco-1.2.4)

# Manually add the include directory for the ArUco library
include_directories(${PROJECT_SOURCE_DIR}/third-party/aruco-1.2.4/src)

catkin_package()

add_executable(tag_detector src/tag_detector_node.cpp)

target_link_libraries(tag_detector ${catkin_LIBRARIES} ${OpenCV_LIBS} aruco)
