cmake_minimum_required(VERSION 3.0.2)
project(path_planner)
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}    -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

add_compile_options(-std=c++17)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
        std_msgs
        tf2_geometry_msgs
        geometry_msgs
        ufomap_msgs
        ufomap_ros
        )
find_package(ufomap REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs nav_msgs ufomap_msgs ufomap_ros visualization_msgs geometry_msgs tf2_geometry_msgs
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node
        ${catkin_LIBRARIES}
        UFO::Map
        )