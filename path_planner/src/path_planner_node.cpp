//
// Created by alsarmi on 04/04/22.
//

#include <path_planner_node.h>

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
void publishPath(NavPath *points, ros::Publisher &pathPublisher) {

  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  nav_msgs::Path path;
  path.header.frame_id = "map";
  tf2::Quaternion q;
  // Convert to ROS nav_msgs/Path format.
  for (auto &point : *points) {
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
int main(int argv, char **argc) {
  ros::init(argv, argc, "path_planner_node");
  ros::NodeHandle nh("");
  auto mapSub =
      nh.subscribe("/ufomap_mapping_server_node/map", 10, mapCallback);
  auto pathPublisher = nh.advertise<nav_msgs::Path>("/rrt/way_points", 1);
  auto pointSub = nh.subscribe("/clicked_point", 10, pointCallback);
  while (!mapAvailable)
    ros::spinOnce();
  auto rrt_planner = RRT<ColorMap, Point, NavPath>(map, 0.05);
  ROS_INFO("Path planner ready!");

  // ufo::math::Vector3 start(1.2045199871063232, 1.3397622108459473,
  //                          1.420783519744873);
  // ufo::math::Vector3 goal(2.2, 1.76, 0.84);
  while (ros::ok()) {
    while (points.size() < 2)
      ros::spinOnce();
    setPoints = false;
    std::cout << "Start point: (x: " << points[0].x() << " y: " << points[0].y()
              << " z: " << points[0].z() << " )" << std::endl;
    std::cout << "Goal point: (x: " << points[1].x() << " y: " << points[1].y()
              << " z: " << points[1].z() << " )" << std::endl;
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