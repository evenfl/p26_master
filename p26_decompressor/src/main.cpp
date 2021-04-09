#include <time.h>
#include <stdio.h>
#include <signal.h>

#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "p26_decompressor/decompressor.h"

void killHandler(int)
{
  ROS_INFO("%s","Shutdown request received.");
  ros::NodeHandle nh;
  ROS_INFO("%s","Terminating nodehandle.");
  nh.shutdown();
  ROS_INFO("%s","Terminating rosnode.");
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "p26_decompressor", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  signal(SIGINT, killHandler);
  signal(SIGTERM, killHandler);

  if(!nh.hasParam("filter_value"))
    ROS_ERROR("%s","Missing _filter_value:=<intensity> parameter! Shutting down...");
  else if(!nh.hasParam("sensor_type"))
    ROS_ERROR("%s","Missing _sensor_type:=<0: Kinect, 1: Velodyne> parameter! Shutting down...");
  else{

    std::string outputTopic;
    std::string inputTopic;
    std::string sensorName;
    std::string sensorFrame;
    float filterValue;
    int sensorType;


    nh.getParam("output_topic", outputTopic);
    nh.getParam("input_topic", inputTopic);
    nh.getParam("filter_value", filterValue);
    nh.getParam("sensor_type", sensorType);
    nh.param<std::string>("sensor_frame", sensorFrame, "world");



    switch(sensorType) {
    case 0: // Kinect

      ROS_INFO("Starting node using KINECT V2");
//      outputTopic = "/p26_lefty/master/"+sensorName+"/kinect_decomp";
//      inputTopic = "/p26_lefty/"+sensorName+"/wp3/kinect_comp";
//      outputTopic = "/p26_lefty/master/kinect_decomp";
//      inputTopic = "/p26_lefty/wp3/kinect_comp";
      break;

    case 1: // Velodyne
      ROS_INFO("Starting node using VELODYNE");
      outputTopic = "/p26_lefty/master/velodyne_decomp";
      inputTopic = "/p26_lefty/wp3/velodyne_comp";
      break;

    } // End switch (sensorType)

    // Start decompressor
    wp3::CloudDecompressor decompressor(outputTopic, inputTopic, sensorFrame, filterValue, false);

    ros::Duration(1.0).sleep();

    ros::spin();
  }

  if(ros::ok()){
    nh.shutdown();
    ros::shutdown();
  }


  ROS_INFO("%s","Shutdown complete.");
  return 0;
}
