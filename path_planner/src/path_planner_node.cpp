//
// Created by alsarmi on 04/04/22.
//

#include "path_planner_node.h"

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
void publishPath(NavPath *points_, ros::Publisher &pathPublisher) {

  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  nav_msgs::Path path;
  path.header.frame_id = "map";
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
    path.poses.emplace_back(pose);
  }
  while (pathPublisher.getNumSubscribers() < 1)
    sleep(1);
  path.header.stamp = ros::Time::now();
  pathPublisher.publish(path);
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
int main(int argv, char **argc) {
  ros::init(argv, argc, "path_planner_node");
  ros::NodeHandle nh("");

  auto mapSub =
      nh.subscribe("/ufomap_mapping_server_node/map", 10, mapCallback);
  auto pathPublisher = nh.advertise<nav_msgs::Path>("/rrt/way_points", 1);
  auto pointSub = nh.subscribe("/clicked_point", 10, pointCallback);
  auto markerPub = nh.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker_array", 1);

  while (!mapAvailable)
    ros::spinOnce();
  auto rrt_planner =
      planner::RRT::RRT<ColorMap, Sphere, Point, NavPath>(&map, 0.05);
  ROS_INFO("Path planner ready!");

  while (ros::ok()) {
    while (points.size() < 2)
      ros::spinOnce();
    setPoints = false;
    std::cout << "Start point: (x: " << points[0].x() << " y: " << points[0].y()
              << " z: " << points[0].z() << " )" << std::endl;
    std::cout << "Goal point: (x: " << points[1].x() << " y: " << points[1].y()
              << " z: " << points[1].z() << " )" << std::endl;

    publishMarkers(markerPub);

    auto path = rrt_planner.computePath(points[0], points[1]);

    if (path != nullptr) {
      ROS_INFO("Path planner  found path!");
      publishPath(path, pathPublisher);
    } else {
      ROS_WARN("NO PATH GENERATED!");
    }
    points.clear();
    setPoints = true;
  }
  ros::spin();
  return 0;
}