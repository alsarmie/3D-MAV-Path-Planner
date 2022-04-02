//
// Created by alsarmi on 31/03/22.
//

#include "ply_publisher.h"
#include <iostream>
#include <utility>
// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//! Class definitions and implementation
/** Constructor */
Ply_publisher::Ply_publisher(ros::NodeHandle *nodeHandle, std::string frameID,
                             std::string rosTopic, ros::Duration pubRate)
    : handle(nodeHandle), pointCloudFrameID(std::move(frameID)),
      pointCloudTopic(std::move(rosTopic)), publishRate(pubRate),
      pointCloudMessage(std::make_unique<sensor_msgs::PointCloud2>()) {

  pointCloudPublisher = nodeHandle->advertise<sensor_msgs::PointCloud2>(
      pointCloudTopic, 1, true); // As the point cloud  can be quite memory
                                 // intensive, we will set the queue at 1.

  publishTimer = nodeHandle->createTimer(
      publishRate, &Ply_publisher::publishingCallback, this);
  ROS_INFO("Point Cloud Publisher created!");
}
/** Move semantics*/
Ply_publisher::Ply_publisher(Ply_publisher &&src) noexcept { swap(src, *this); }

Ply_publisher &Ply_publisher::operator=(Ply_publisher &&src) noexcept {
  std::cout << "Point Cloud Publisher move assignment operation.\n";
  swap(src, *this);
  return *this;
}

/** Member functions*/
Ply_publisher Ply_publisher::setup(ros::NodeHandle *nodeHandle) {
  // Point cloud file's absolute path.
  std::string filePath;
  // Ros topic to publish to.
  std::string pointCloudTopic;
  // Point cloud frame ID.
  std::string pointCloudFrameID;
  // Publishing rate
  double rate;
  // For timed callback3
  ros::Duration publishRate;

  // Load from launch file
  if (!nodeHandle->getParam("path", filePath)) {
    ROS_WARN(
        "Path to *.ply file not provided in launch file or argument "
        "path:=your_path_to_ply_file when running as a single node! Assigning "
        "default demo file!");
    filePath = "../models/room.ply";
  }
  if (!nodeHandle->getParam("topic", pointCloudTopic)) {
    ROS_WARN("ROS topic not provided in launch file or argument "
             "pointCloudTopic:=your_ros_topic when running as a single node! "
             "Assigning "
             "default topic!");
    pointCloudTopic = "/point_cloud";
  }
  if (!nodeHandle->getParam("frame_id", pointCloudFrameID)) {
    ROS_WARN(
        "Point cloud frame ID not provided in launch file or argument "
        "pointCloudFrameID:=frameID when running as a single node! Assigning "
        "default frameID!");
    pointCloudFrameID = "/t265_d400_base";
  }
  if (!nodeHandle->getParam("publish_rate", rate)) {
    ROS_WARN("Publish rate not provided in launch file or argument "
             "publish_rate:=rate when running as a single node! Assigning "
             "default publishing rate!");
    rate = 1.0;
  }
  publishRate.fromSec(1.0 / rate);
  Ply_publisher publisher(std::move(nodeHandle), std::move(pointCloudFrameID),
                          std::move(pointCloudTopic), std::move(publishRate));
  if (!publisher.loadFile(filePath)) {
    auto message = "Could not load provided file in path:" + filePath;
    ROS_ERROR("Could not load file from provided path!");
    ros::requestShutdown();
  }

  return publisher;
}
void Ply_publisher::publishingCallback(const ros::TimerEvent &timerEvent) {
  if (!publish())
    ROS_ERROR("Could not publish point cloud!");
}
bool Ply_publisher::loadFile(const std::string &path) {
  if (path.find(".ply") == std::string::npos) {
    ROS_WARN("Provided file format is different from .ply!");
    return false;
  }
  // Load file
  std::unique_ptr<pcl::PointCloud<pcl::PointXYZRGB>> pointCloud =
      std::make_unique<pcl::PointCloud<pcl::PointXYZRGB>>();
  // Use PCL library to load the .ply file
  if (pcl::io::loadPLYFile(path, *pointCloud) != 0)
    return false;
  // Transform to ROS message
  pcl::toROSMsg(*pointCloud, *pointCloudMessage);

  ROS_INFO("Successfully loaded point cloud data!");
  return true;
}
bool Ply_publisher::publish() {
  pointCloudMessage->header.stamp = ros::Time::now();
  pointCloudMessage->header.frame_id = pointCloudFrameID;
  if (pointCloudPublisher.getNumSubscribers() > 0) {
    pointCloudPublisher.publish(*pointCloudMessage);
    // ROS_INFO("Published point cloud.");
  }
  return true;
}