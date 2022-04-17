//
// Created by alsarmi on 07/04/22.
//

#ifndef PATH_PLANNER_PATH_PLANNER_NODE_H
#define PATH_PLANNER_PATH_PLANNER_NODE_H
#include "ros/ros.h"
#include <ufo/map/occupancy_map_color.h>
// UFOMap ROS msg
#include <ufomap_msgs/UFOMapStamped.h>
// To convert between UFO and ROS
#include "rrt_planner.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ufomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

static ufo::map::OccupancyMapColor map(0.05);
bool mapAvailable = false;
bool setPoints = true;
std::vector<ufo::math::Vector3> points;

#endif // PATH_PLANNER_PATH_PLANNER_NODE_H