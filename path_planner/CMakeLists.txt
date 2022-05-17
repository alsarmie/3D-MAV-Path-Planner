cmake_minimum_required(VERSION 3.0.2)
project(path_planner)
IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
ENDIF ()

message("Build type: " ${CMAKE_BUILD_TYPE})


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}    -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
add_compile_options(-std=c++17 -pthread)

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