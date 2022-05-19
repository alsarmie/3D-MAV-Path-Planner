
#include "ply_publisher.h"

/**
 * @brief Point cloud publisher node main file.
 * @param argc The number of arguments passed to the program.
 * @param argv The arguments to pass to the node, currently none.
 *
 * @return Exit point.
 */
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cloud_publisher");
  ros::NodeHandle nh("~");
  // Call point cloud publisher, share the Node handle through reference.
  PlyPublisher cloudPublisher = PlyPublisher::setup(&nh);
  cloudPublisher.publish(); // Publish
  ros::spin();
  return 0;
}