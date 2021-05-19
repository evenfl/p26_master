#pragma once

#include "main.h"

pcl::PointCloud<PointT>::Ptr segment(const pcl::PointCloud<PointT>::Ptr& input, pcl::ModelCoefficients::Ptr coefficients_cylinder);

pcl::PointCloud<pcl::PointXYZ>::Ptr
passThroughFilterSphere(pcl::PointCloud<PointT>::Ptr& cloud,
pcl::PointXYZ sphereCenterPoint, const double radius, bool remove_outside);
