cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_publisher)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}    -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(BUILD_DOC "Build documentation" ON)

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
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