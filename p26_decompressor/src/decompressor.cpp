/*
 * decompressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "p26_decompressor/decompressor.h"

namespace wp3 {

// Constructor
CloudDecompressor::CloudDecompressor(std::string outputCloudTopic, std::string inputMsgTopic, std::string sensorFrame, const float intensityLimit, const bool showStatistics) :
  decompressedCloud(new PointCloudXYZI ()),
  outputCloud(new PointCloudXYZI ()),
  ptfilter(false),
  sensorFrame(sensorFrame),
  intensityLimit(intensityLimit),
  showStatistics(showStatistics),
  logFile("/home/sfi/decompressorlog.txt"),
  pointCloudDecoder(showStatistics)
{
  if(showStatistics){
    logStream.open(logFile.c_str());
    logStream << "Time" << std::endl;
  }

  sub = nh.subscribe<p26_decompressor::comp_msg>(inputMsgTopic, 1, &wp3::CloudDecompressor::roscallback, this);
  pub = nh.advertise<PointCloudXYZI>(outputCloudTopic, 1);
}

CloudDecompressor::~CloudDecompressor(){
  if(showStatistics)
    logStream.close();
}

// Callback for ROS subscriber
void CloudDecompressor::roscallback(const p26_decompressor::comp_msg::ConstPtr & msg){

  // Stream for storing serialized compressed point cloud
  std::stringstream compressedData;

  // Retreive data from message
  compressedData << msg->data;
  compressedData.flush();
  
  // Decode stream to point cloud
  pointCloudDecoder.decodePointCloud (compressedData, decompressedCloud);

  if(decompressedCloud->width > 0){
    // Filter point cloud based on intensity
    ptfilter.setInputCloud (decompressedCloud);
    ptfilter.setFilterFieldName ("intensity");
    ptfilter.setFilterLimits (intensityLimit, FLT_MAX);
    ptfilter.filter (*outputCloud);

    // Publish the decompressed cloud
    outputCloud->header.frame_id = sensorFrame;
    outputCloud->header.seq = msg->header.seq;
    pcl_conversions::toPCL(msg->header.stamp, outputCloud->header.stamp);
    pub.publish(outputCloud);
  }

}

}
