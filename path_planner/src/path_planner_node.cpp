//
// Created by alsarmi on 04/04/22.
//

#include "path_planner_node.h"

int main(int argv, char **argc) {

  // Initialize  Node
  ros::init(argv, argc, "path_planner_node");
  ros::NodeHandle nh("~");
  // Subscribers and Publishers
  auto mapSub =
      nh.subscribe("/ufomap_mapping_server_node/map", 10, mapCallback);
  auto pathPublisher = nh.advertise<nav_msgs::Path>("/rrt/way_points", 1);
  auto pathIntPublisher =
      nh.advertise<nav_msgs::Path>("/rrt/way_points_int", 1);
  auto pointSub = nh.subscribe("/clicked_point", 10, pointCallback);
  auto markerPub = nh.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker_array", 1);
  // Create variant
  std::variant<RRT, BRRT, IBRRT> rrt_planner;
  // Path smoothing object
  auto catmullRomSpline = globalPlanner::CatmullRomSpline<Point, NavPath>();
  // Wait for map before trying to compute a path!
  while (!mapAvailable)
    ros::spinOnce();
  // Set the RRT planner
  setPlannerType(nh, rrt_planner);
  // Until shutdown is executed, retrieve 3D points from Rviz provided by user.
  while (ros::ok()) {
    path = nullptr;
    getRVIZPoints(); // Waiting function, returns when the pointCallback
                     // receives new points.
    publishPointsMarkers(markerPub);

    std::visit(overload{[&](RRT &planner) {
                          path = planner.computePath(points[0], points[1]);
                        },
                        [&](BRRT &planner) {
                          path = planner.computePath(points[0], points[1]);
                        },
                        [&](IBRRT &planner) {
                          path = planner.computePath(points[0], points[1]);
                        }},
               rrt_planner);

    if (path != nullptr) {
      auto interpolatedPath = catmullRomSpline.interpolate(10.0, path);
      if (!interpolatedPath.empty()) {
        ROS_INFO("Path globalPlanner  found path!");
        publishPath(path, pathPublisher);
        publishInterpolatedPath(&interpolatedPath, pathIntPublisher);
        ROS_INFO("Path publish!");
      } else {
        ROS_ERROR("NO PATH GENERATED!");
      }
    } else {
      ROS_ERROR("NO PATH GENERATED!");
    }
  }
  /*  ros::spin();*/
  return 0;
}