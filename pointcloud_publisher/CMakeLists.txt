cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_publisher)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        pcl_ros
        tf2
        tf2_ros
        )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs tf2 tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
add_executable(
        cloud_publisher
        src/cloud_publisher_node.cpp
        src/ply_publisher.cpp
)
target_link_libraries(
        cloud_publisher ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/marker_map_reference.py
        scripts/map_to_odom_transformation.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )