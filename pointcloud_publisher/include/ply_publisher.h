/**
 *  @author  Alejandro Sarmiento.
 *  @brief Header definition of .ply point cloud reader and publisher.
 */
#ifndef RGBD_MAPPING_PLY_PUBLISHER_H
#define RGBD_MAPPING_PLY_PUBLISHER_H
#include "pointcloud_publisher.h"

class PlyPublisher : public Publisher {
public:
  // Constructor
  PlyPublisher(ros::NodeHandle *nodeHandle, std::string frameID,
                std::string rosTopic, ros::Duration pubRate);
  // Move constructor and move assignment operator
  PlyPublisher(PlyPublisher &&src) noexcept;
  PlyPublisher &operator=(PlyPublisher &&src) noexcept;
  // Destructor
  ~PlyPublisher() override = default;
  // Copy constructor and Copy assignment are forbidden.
  PlyPublisher(PlyPublisher &src) = delete;
  PlyPublisher &operator=(PlyPublisher &src) = delete;

  // Setup member function, we will wrap the constructor with it, so that we can
  // keep the constructor atomic with low complexity.
  static PlyPublisher setup(ros::NodeHandle *nodeHandle);
  // Member functions
  bool publish() override;
  bool loadFile(const std::string &path) override;

private:
  // For Move Semantics
  /**
   * A friend function that is used to swap the values of the member variables
   * of the class.
   * @param src Source ply_publisher class instance.
   * @param dst  Destination ply_publisher class instance.
   */
  friend void swap(PlyPublisher &src, PlyPublisher &dst) {
    using std::swap;
    swap(src.handle, dst.handle);
    swap(src.publishRate, dst.publishRate);
    swap(src.publishTimer, dst.publishTimer);
    swap(src.pointCloudFrameID, dst.pointCloudFrameID);
    swap(src.pointCloudTopic, dst.pointCloudTopic);
    swap(src.pointCloudMessage, dst.pointCloudMessage);
    swap(src.pointCloudPublisher, dst.pointCloudPublisher);
  }

  // Member functions
  void publishingCallback(const ros::TimerEvent &timerEvent);
  // Handle
  ros::NodeHandle *handle = nullptr;
  // Point cloud frame ID.
  std::string pointCloudFrameID;
  // ROS publisher.
  ros::Publisher pointCloudPublisher;
  // Ros topic to publish to.
  std::string pointCloudTopic;
  // Pointer to point cloud message
  std::unique_ptr<sensor_msgs::PointCloud2> pointCloudMessage;
  // To constantly publish the cloud point we require a ros::Timer and a
  // ros::Duration
  ros::Timer publishTimer;
  ros::Duration publishRate;
};

#endif // RGBD_MAPPING_PLY_PUBLISHER_H