//
// Created by alsarmi on 07/04/22.
//

#ifndef PATH_PLANNER_PATH_PLANNER_NODE_H
#define PATH_PLANNER_PATH_PLANNER_NODE_H

#include "b_rrt_star_planner.h"
#include "ib_rrt_star_planner.h"
#include "ros/ros.h"
#include "rrt_star_planner.h"
#include <ufo/map/occupancy_map_color.h>
// UFOMap ROS msg
#include <ufomap_msgs/UFOMapStamped.h>
// To convert between UFO and ROS
#include "path_smoothing.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ufomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// Tags
using Sphere = ufo::geometry::Sphere;
using Point = ufo::math::Vector3;
using ColorMap = ufo::map::OccupancyMapColor;
using NavPath = std::deque<Point>;
using IBRRT = globalPlanner::RRT::IBRRT<ColorMap, Sphere, Point, NavPath>;
using BRRT = globalPlanner::RRT::BRRT<ColorMap, Sphere, Point, NavPath>;
using RRT = globalPlanner::RRT::RRTStar<ColorMap, Sphere, Point, NavPath>;
// Path planner variables
// Boilerplate code for variant/visit
template <class... Ts> struct overload : Ts... { using Ts::operator()...; };
template <class... Ts>
overload(Ts...) -> overload<Ts...>; // line not needed in C++20...

int type{0};
bool mapAvailable = false;
bool setPoints = true;
NavPath *path;
static ufo::map::OccupancyMapColor map(0.05);
std::vector<ufo::math::Vector3> points;

// Functions & Callbacks

void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg) {

  if (ufomap_msgs::msgToUfo(msg->map, map)) {
    ROS_INFO_ONCE("Map conversion successful!");
    mapAvailable = true;

  } else {
    ROS_WARN("Map conversion failed!");
    mapAvailable = false;
  }
}
void pointCallback(geometry_msgs::PointStamped::ConstPtr const &msg) {
  if (setPoints) {
    points.emplace_back(
        ufo::math::Vector3(msg->point.x, msg->point.y, msg->point.z));
  }
}

void publishInterpolatedPath(NavPath *points_, ros::Publisher &pathPublisher) {

  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  nav_msgs::Path path_;
  path_.header.frame_id = "map";
  tf2::Quaternion q;

  // Convert to ROS nav_msgs/Path format.
  for (auto &point : *points_) {

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    point /= point.norm();
    yaw = -atan2(point.y(), point.x());
    pitch = -asin(point.y());
    q.setRPY(roll, pitch, yaw);
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    path_.poses.emplace_back(pose);
  }
  while (pathPublisher.getNumSubscribers() < 1)
    sleep(1);
  path_.header.stamp = ros::Time::now();
  pathPublisher.publish(path_);
}
void publishPath(NavPath *points_, ros::Publisher &pathPublisher) {

  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  nav_msgs::Path path_;
  path_.header.frame_id = "map";
  tf2::Quaternion q;
  points_->pop_back();
  points_->pop_front();
  // Convert to ROS nav_msgs/Path format.
  for (auto &point : *points_) {

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = point.z();
    point /= point.norm();
    yaw = -atan2(point.y(), point.x());
    pitch = -asin(point.y());
    q.setRPY(roll, pitch, yaw);
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    path_.poses.emplace_back(pose);
  }
  while (pathPublisher.getNumSubscribers() < 1)
    sleep(1);
  path_.header.stamp = ros::Time::now();
  pathPublisher.publish(path_);
}
void publishMarkers(ros::Publisher &markerPublisher) {
  int cnt{0};
  visualization_msgs::MarkerArray markerArray;
  for (auto &point : points) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = cnt++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = point.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.header.stamp = ros::Time();
    markerArray.markers.push_back(marker);
  }
  markerPublisher.publish(markerArray);
}
globalPlanner::RRT::RRTStar<ColorMap, Sphere, Point, NavPath>
setUpRRTPlanner(ros::NodeHandle &nh) {
  ufo::math::Vector3 min_;
  ufo::math::Vector3 max_;
  double delta_{0.0};
  double searchRadius_{0.0};
  double thresh_{0.0};
  double radius_{0.0};
  double eps_{0.0};
  nh.getParam("step", delta_);
  nh.getParam("search_radius", searchRadius_);
  nh.getParam("threshold", thresh_);
  nh.getParam("radius", radius_);
  nh.getParam("eps", eps_);
  nh.getParam("x_min", min_[0]);
  nh.getParam("y_min", min_[1]);
  nh.getParam("z_min", min_[2]);
  nh.getParam("x_max", max_[0]);
  nh.getParam("y_max", max_[1]);
  nh.getParam("z_max", max_[2]);
  return {&map, min_, max_, delta_, eps_, thresh_, searchRadius_, radius_};
}
globalPlanner::RRT::BRRT<ColorMap, Sphere, Point, NavPath>
setUpBRRTPlanner(ros::NodeHandle &nh) {
  ufo::math::Vector3 min_;
  ufo::math::Vector3 max_;
  double delta_{0.0};
  double searchRadius_{0.0};
  double radius_{0.0};
  int iterations_{0};
  nh.getParam("step", delta_);
  nh.getParam("radius", radius_);
  nh.getParam("search_radius", searchRadius_);
  nh.getParam("iterations", iterations_);
  if (iterations_ > 500000) {
    ROS_WARN("Maximum number of iterations exceeded, setting value to 500,000");
    iterations_ = 500000;
  }

  nh.getParam("x_min", min_[0]);
  nh.getParam("y_min", min_[1]);
  nh.getParam("z_min", min_[2]);
  nh.getParam("x_max", max_[0]);
  nh.getParam("y_max", max_[1]);
  nh.getParam("z_max", max_[2]);

  return {&map, min_, max_, delta_, searchRadius_, radius_, iterations_};
}
globalPlanner::RRT::IBRRT<ColorMap, Sphere, Point, NavPath>
setUpIBRRTPlanner(ros::NodeHandle &nh) {
  ufo::math::Vector3 min_(-5.0, -5.0, -3.0);
  ufo::math::Vector3 max_(5.0, 5.0, 3.0);
  double delta_{0.20};
  double searchRadius_{1.00};
  double radius_{0.05};
  int iterations_{10000};
  nh.getParam("step", delta_);
  nh.getParam("radius", radius_);
  nh.getParam("search_radius", searchRadius_);
  nh.getParam("iterations", iterations_);
  if (iterations_ > 500000) {
    ROS_WARN("Maximum number of iterations exceeded, setting value to 500,000");
    iterations_ = 500000;
  }

  nh.getParam("x_min", min_[0]);
  nh.getParam("y_min", min_[1]);
  nh.getParam("z_min", min_[2]);
  nh.getParam("x_max", max_[0]);
  nh.getParam("y_max", max_[1]);
  nh.getParam("z_max", max_[2]);

  return {&map, min_, max_, delta_, searchRadius_, radius_, iterations_};
}
void setPlannerType(ros::NodeHandle &nh,
                    std::variant<RRT, BRRT, IBRRT> &planner) {
  nh.getParam("planner", type);
  // For debugging:
  /*type = 2;*/
  switch (type) {
  case 0: // RRT*
    planner = setUpRRTPlanner(nh);
    ROS_INFO("RRT* path globalPlanner ready!");
    break;
  case 1: // B-RRT*
    planner = setUpBRRTPlanner(nh);
    ROS_INFO("B-RRT* path globalPlanner ready!");
    break;
  case 2: // IB-RRT*
    planner = setUpIBRRTPlanner(nh);
    ROS_INFO("IB-RRT* path globalPlanner ready!");
    break;
  default:
    ROS_WARN("No planner selected! Using RRT* as default option!");
    planner = setUpRRTPlanner(nh);
    ROS_INFO("RRT* path globalPlanner ready!");
  }
}
void publishPointsMarkers(ros::Publisher &pub) {
  std::cout << "Start point: (x: " << points[0].x() << " y: " << points[0].y()
            << " z: " << points[0].z() << " )" << std::endl;
  std::cout << "Goal point: (x: " << points[1].x() << " y: " << points[1].y()
            << " z: " << points[1].z() << " )" << std::endl;
  publishMarkers(pub);
}
void getRVIZPoints() {
  points.clear();
  setPoints = true;
  while (points.size() < 2)
    ros::spinOnce();
  setPoints = false;
}
#endif // PATH_PLANNER_PATH_PLANNER_NODE_H