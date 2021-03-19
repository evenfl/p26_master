/*
 * decompressor.h
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#ifndef DECOMPRESSOR_H_
#define DECOMPRESSOR_H_

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include "octree_decompression.h"
#include "p26_decompressor/comp_msg.h"



typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudDecompression Decompressor;

namespace wp3 {

class CloudDecompressor
{
public:
  // Constructor
  CloudDecompressor(std::string outputCloudTopic, std::string inputMsgTopic, std::string sensorFrame, const float intensityLimit, const bool showStatistics);

  // Deconstrucor
  ~CloudDecompressor();

  // Callback for PointCloudXYZ subscriber
  void roscallback(const p26_decompressor::comp_msg::ConstPtr & msg);

private:

  // ROS variables
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;

  // Pointers to temporary point clouds
  PointCloudXYZI::Ptr decompressedCloud;
  PointCloudXYZI::Ptr outputCloud;

  Decompressor pointCloudDecoder;

  // pointcloud frame
  std::string sensorFrame;


  // Passthrough filter
  float intensityLimit;
  pcl::PassThrough<PointType_out> ptfilter; // Initializing with true will allow us to extract the removed indices

  // Logging
  bool showStatistics;
  std::string logFile;
  std::ofstream logStream;

};

}


#endif /* DECOMPRESSOR_H_ */
