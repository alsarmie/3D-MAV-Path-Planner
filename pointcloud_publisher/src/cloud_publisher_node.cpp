//
// Created by alsarmi on 01/04/22.
//
#include "ply_publisher.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cloud_publisher");
  ros::NodeHandle nh("~");
  // Call point cloud publisher, share the Node handle through reference.
  Ply_publisher cloudPublisher = Ply_publisher::setup(&nh);
  cloudPublisher.publish();
  // Publish

  ros::spin();
  return 0;
}