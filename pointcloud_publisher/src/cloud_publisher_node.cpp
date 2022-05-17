
#include "ply_publisher.h"

/**
 *  Created by Alejandro Sarmiento
 *
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments to pass to the node, currently none.
 *
 * @return Exit point.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cloud_publisher");
  ros::NodeHandle nh("~");
  // Call point cloud publisher, share the Node handle through reference.
  Ply_publisher cloudPublisher = Ply_publisher::setup(&nh);
  cloudPublisher.publish(); // Publish
  ros::spin();
  return 0;
}