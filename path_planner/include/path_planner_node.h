//
// Created by Alejandro Sarmiento on 07/04/22.
//

#ifndef PATH_PLANNER_PATH_PLANNER_NODE_H
#define PATH_PLANNER_PATH_PLANNER_NODE_H

#include "b_rrt_star_planner.h"
#include "ib_rrt_star_planner.h"
#include "parallel_ib_rrt_star_planner.h"
#include "path_smoothing.h"
#include "ros/ros.h"
#include "rrt_star_planner.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ufo/map/occupancy_map_color.h> // UFOMap ROS msg
#include <ufomap_msgs/UFOMapStamped.h>   // To convert between UFO and ROS
#include <ufomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace globalPlanner {
// Tags to improve readability.
using Sphere = ufo::geometry::Sphere;
using Point = ufo::math::Vector3;
using Points = std::vector<ufo::math::Vector3>;
using ColorMap = ufo::map::OccupancyMapColor;
using NavPath = std::deque<Point>;
using IBRRT = RRT::IBRRT<ColorMap, Sphere, Point, NavPath>;
using PIBRRT = RRT::PIBRRT<ColorMap, Sphere, Point, NavPath>;
using BRRT = RRT::BRRT<ColorMap, Sphere, Point, NavPath>;
using RRTS = RRT::RRTStar<ColorMap, Sphere, Point, NavPath>;
// Boilerplate code for variant/visit
template <class... Ts> struct overload : Ts... { using Ts::operator()...; };
template <class... Ts> overload(Ts...) -> overload<Ts...>; //
class pathPlanner {
public:
  explicit pathPlanner(ros::NodeHandle &);
  void run();

protected:
  void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg);
  void pointCallback(geometry_msgs::PointStamped::ConstPtr const &msg);
  void publishInterpolatedPath();
  void publishPath();
  void getRvizPoints();
  void publishPointsMarkers();
  void publishMarkers();
  void setPlannerType();

private:
  ros::NodeHandle &nh;
  ros::Subscriber mapSubscriber;
  ros::Subscriber pointSubscriber;
  ros::Publisher pathPublisher;
  ros::Publisher pathIntPublisher;
  ros::Publisher markerPublisher;
  // Path planner variables
  int type{0};
  bool mapAvailable = false;
  bool setPoints = true;
  NavPath *path{};
  NavPath interpolatedPath{};
  ColorMap map{0.05};
  Points points{};
  // Create variant
  std::variant<RRTS, BRRT, IBRRT, PIBRRT> rrt_planner{};
  globalPlanner::CatmullRomSpline<Point, NavPath> catmullRomSpline{};
  // For planner
  ufo::math::Vector3 min_{-3.0, -3.0, 0.0};
  ufo::math::Vector3 max_{3.0, 3.0, 2.0};
  double delta_{0.0};
  double searchRadius_{0.0};
  double radius_{0.0};
  double thresh_{0.0};
  double eps_{0.0};
  int iterations_{0};
  int workers_{10};
};
/**
 * This is the constructor of the class `pathPlanner`. It is initializing the
 * class variables and setting up the ROS subscribers and publishers.
 * @param handle Reference to Ros node handle.
 */
pathPlanner::pathPlanner(ros::NodeHandle &handle) : nh(handle) {
  // Subscribers and Publishers
  mapSubscriber = nh.subscribe("/ufomap_mapping_server_node/map", 10,
                               &pathPlanner::mapCallback, this);
  pathPublisher = nh.advertise<nav_msgs::Path>("/rrt/way_points", 1);
  pathIntPublisher = nh.advertise<nav_msgs::Path>("/rrt/way_points_int", 1);
  pointSubscriber =
      nh.subscribe("/clicked_point", 10, &pathPlanner::pointCallback, this);
  markerPublisher = nh.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker_array", 1);
  nh.getParam("planner", type);
  nh.getParam("step", delta_);
  nh.getParam("radius", radius_);
  nh.getParam("search_radius", searchRadius_);
  nh.getParam("eps", eps_);
  nh.getParam("threshold", thresh_);
  nh.getParam("x_min", min_[0]);
  nh.getParam("y_min", min_[1]);
  nh.getParam("z_min", min_[2]);
  nh.getParam("x_max", max_[0]);
  nh.getParam("y_max", max_[1]);
  nh.getParam("z_max", max_[2]);
  nh.getParam("workers", workers_);
  nh.getParam("iterations", iterations_);
  if (iterations_ > 500000) {
    ROS_WARN("Maximum number of iterations exceeded, setting value to 500,000");
    iterations_ = 500000;
  }
  // Set the RRT planner
  setPlannerType();
}

/**
 * @brief This is the main function of the class. It is waiting for the
 * map to be available, and then it is waiting for the user to provide two points
 * in Rviz. Once the points are provided, the planner is called and the path is
 * published.
 */
void pathPlanner::run() {
  // Wait for map before trying to compute a path!
  while (!mapAvailable)
    ros::spinOnce();
  // Until shutdown is executed, retrieve 3D points from Rviz provided by user.
  while (ros::ok()) {
    path = nullptr;
    getRvizPoints(); // Waiting function, returns when the pointCallback
                     // receives new points.
    publishPointsMarkers();
    // path = rrt_planner.computePath(points[0], points[1]);
    std::visit(overload{[&](RRTS &planner) {
                          path = planner.computePath(points[0], points[1]);
                        },
                        [&](BRRT &planner) {
                          path = planner.computePath(points[0], points[1]);
                        },
                        [&](IBRRT &planner) {
                          path = planner.computePath(points[0], points[1]);
                        },
                        [&](PIBRRT &planner) {
                          path = planner.computePath(points[0], points[1]);
                        }},
               rrt_planner);

    if (path != nullptr) {
      interpolatedPath = catmullRomSpline.interpolate(10.0, path);
      if (!interpolatedPath.empty()) {
        ROS_INFO("Path globalPlanner found path!");
        publishPath();
        publishInterpolatedPath();
        ROS_INFO("Path publish!");
      } else {
        ROS_ERROR("NO PATH GENERATED!");
      }
    } else {
      ROS_ERROR("NO PATH GENERATED!");
    }
  }
}
// Functions & Callbacks
/**
 * @brief Converting the map from the UFOMap format to the UFO format.
 * @param msg UfoMap ros message.
 */
void pathPlanner::mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg) {
  if (ufomap_msgs::msgToUfo(msg->map, map)) {
    ROS_INFO_ONCE("Map conversion successful!");
    mapAvailable = true;

  } else {
    ROS_WARN("Map conversion failed!");
    mapAvailable = false;
  }
}

/**
 * @brief This function is the callback for the subscriber to the
 `/clicked_point` topic. It is receiving the points from Rviz and storing them
 in the `points` vector.
 * @param msg
 */
void pathPlanner::pointCallback(
    geometry_msgs::PointStamped::ConstPtr const &msg) {
  ROS_INFO("Setting points!");
  if (setPoints) {
    points.emplace_back(
        ufo::math::Vector3(msg->point.x, msg->point.y, msg->point.z));
  }
}
/**
 * @brief This function waits for Rviz points to be available.
 */
void pathPlanner::getRvizPoints() {
  points.clear();
  setPoints = true;
  ROS_INFO("Waiting for RVIZ points!");
  while (points.size() < 2)
    ros::spinOnce();
  setPoints = false;
}
/**
 *  @brief Publish start and goal points to Rviz as visible markers.
 */
void pathPlanner::publishPointsMarkers() {
  std::cout << "Start point: (x: " << points[0].x() << " y: " << points[0].y()
            << " z: " << points[0].z() << " )" << std::endl;
  std::cout << "Goal point: (x: " << points[1].x() << " y: " << points[1].y()
            << " z: " << points[1].z() << " )" << std::endl;
  publishMarkers();
}
/**
 * @brief This function converts the interpolated path from the UFO format
to the ROS nav_msgs/Path format. After conversion, the path is published to
/rrt/way_points_int.
 */
void pathPlanner::publishInterpolatedPath() {
  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  nav_msgs::Path path_;
  path_.header.frame_id = "map";
  tf2::Quaternion q;

  // Convert to ROS nav_msgs/Path format.
  for (auto &point : interpolatedPath) {
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
  while (pathIntPublisher.getNumSubscribers() < 1)
    ros::spinOnce(); // sleep(1);
  path_.header.stamp = ros::Time::now();
  pathIntPublisher.publish(path_);
  interpolatedPath.clear();
}

/**
 * @brief Publishes the path to the topic /rrt/way_points.
 */
void pathPlanner::publishPath() {
  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  nav_msgs::Path path_;
  path_.header.frame_id = "map";
  tf2::Quaternion q;
  path->pop_back();
  path->pop_front();
  //  Convert to ROS nav_msgs/Path format.
  for (auto &point : *path) {
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
    ros::spinOnce();
  path_.header.stamp = ros::Time::now();
  pathPublisher.publish(path_);
}
/**
 * @brief Publishes the start and goal markers to the topic
 * /visualization_marker_array.
 */
void pathPlanner::publishMarkers() {
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
/**
 * @brief Set the planner type: RRT *, B-RRT * ,IB-RRT * and Parallel
 * IB-RRT *. The type is read from ros launch file or parameter server.
 */
void pathPlanner::setPlannerType() {
  switch (type) {
  case 0: // RRT*
    rrt_planner =
        RRTS(&map, min_, max_, delta_, eps_, thresh_, searchRadius_, radius_);
    ROS_INFO("RRT* path globalPlanner ready!");
    break;
  case 1: // B-RRT*
    rrt_planner =
        BRRT(&map, min_, max_, delta_, searchRadius_, radius_, iterations_);
    ROS_INFO("B-RRT* path globalPlanner ready!");
    break;
  case 2: // IB-RRT*
    rrt_planner =
        IBRRT(&map, min_, max_, delta_, searchRadius_, radius_, iterations_);
    ROS_INFO("IB-RRT* path globalPlanner ready!");
    break;
  case 3:
    rrt_planner = PIBRRT(&map, min_, max_, delta_, searchRadius_, radius_,
                         iterations_, workers_);
    ROS_INFO("Parallel IB-RRT* path globalPlanner ready!");
    break;
  default:
    ROS_WARN("No planner selected! Using RRT* as default option!");
    rrt_planner =
        RRTS(&map, min_, max_, delta_, eps_, thresh_, searchRadius_, radius_);
    ROS_INFO("RRT* path globalPlanner ready!");
  }
}
} // namespace globalPlanner
#endif // PATH_PLANNER_PATH_PLANNER_NODE_H