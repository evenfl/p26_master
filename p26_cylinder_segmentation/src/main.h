#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/distances.h>
#include "pcl_ros/point_cloud.h"
#include <boost/foreach.hpp>
#include <iostream>
#include <string>
#include <fstream> //For saving to text file
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <math.h>       /* round, floor, ceil, trunc */
#include <cmath>      /* std::abs */
#include "geometry_msgs/Point.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


//const double cylinderDiameter = 0.169; // Yellow pipe in lab
const double cylinderDiameter = 0.26;
//const double cylinderDiameter = 0.16;
const double cylinderRadius = cylinderDiameter/2;
//const double cylinderLength = 0.53; // Yellow pipe in lab
const double cylinderLength = 1.2;

const float x_min = 4.2;
const float x_max = 6.0;
const float y_min = 1.0;
const float y_max = 10.0;
const float z_min = 0.05;
const float z_max = 2.0;


struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };

