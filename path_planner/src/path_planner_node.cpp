/**
 *@mainpage  Path Planner Node
 *@author  Alejandro Sarmiento.
 *@file  path_planner_node.cpp
 *@brief Path planner Ros node main file
 */
#include "path_planner_node.h"

/**
 * The main function initializes the ROS node, creates a pathPlanner object, and
 * runs the planner until it is interrupted by the user.
 *
 * @param argv number of arguments
 * @param argc The number of arguments passed to the program
 *
 * @return The return value is the exit status of the program.
 */
int main(int argv, char **argc) {
  // Initialize Node
  ros::init(argv, argc, "path_planner_node");
  ros::NodeHandle nh("~");
  // Create planner object
  globalPlanner::pathPlanner planner(nh);
  // Run planner
  planner.run();
  return 0;
}