//
// Created by alsarmi on 31/03/22.
//

#ifndef RGBD_MAPPING_PLY_PUBLISHER_H
#define RGBD_MAPPING_PLY_PUBLISHER_H
#include "pointcloud_publisher.h"

class Ply_publisher : public Publisher {
public:
  // Constructor
  Ply_publisher(ros::NodeHandle *nodeHandle, std::string frameID,
                std::string rosTopic, ros::Duration pubRate);
  // Move constructor and move assignment operator
  Ply_publisher(Ply_publisher &&src) noexcept;
  Ply_publisher &operator=(Ply_publisher &&src) noexcept;
  // Destructor
  ~Ply_publisher() override = default;
  // Copy constructor and Copy assignment are forbidden.
  Ply_publisher(Ply_publisher &src) = delete;
  Ply_publisher &operator=(Ply_publisher &src) = delete;

  // Setup member function, we will wrap the constructor with it, so that we can
  // keep the constructor atomic with low complexity.
  static Ply_publisher setup(ros::NodeHandle *nodeHandle);
  // Member functions
  bool publish() override;
  bool loadFile(const std::string &path) override;

private:
  // For Move Semantics
  friend void swap(Ply_publisher &src, Ply_publisher &dst) {
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
  // To constantly publish the cloud point we require a ross::Timer and a
  // ros::Duration
  ros::Timer publishTimer;
  ros::Duration publishRate;
};

#endif // RGBD_MAPPING_PLY_PUBLISHER_H