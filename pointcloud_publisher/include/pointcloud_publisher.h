/***
 * @author Alejandro Sarmiento
 * @description This header defines the general class point cloud_publisher that
 * is expected to be inherited on derived classes that read and publish
 * different point cloud formats.
 *
 */
#ifndef RGBD_MAPPING_POINTCLOUD_PUBLISHER_H
#define RGBD_MAPPING_POINTCLOUD_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class Publisher {
public:
  // Constructor
  Publisher() = default;
  // Destructor
  virtual ~Publisher() = default;
  // Member functions
  virtual bool publish() = 0;
  virtual bool loadFile(const std::string &path) = 0;
  // Member functions
  virtual void loadParams(){};
  //
};
#endif // RGBD_MAPPING_POINTCLOUD_PUBLISHER_H