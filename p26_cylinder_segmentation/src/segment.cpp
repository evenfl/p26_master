#include "segment.h"

pcl::PointCloud<PointT>::Ptr segment(const pcl::PointCloud<PointT>::Ptr& input, pcl::ModelCoefficients::Ptr coefficients_cylinder) //const sensor_msgs::PointCloudConstPtr& input)
{

  // All the objects needed
  //pcl::PCDReader reader;
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Datasets
  //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr filter_x (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr filter_y (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr filter_z (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);//, coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  //reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  //std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;
  //std::cerr << "PointCloud has: " << input->size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  //pass.setInputCloud (cloud);
  pass.setInputCloud (input);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_min, x_max); //5.5 works
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_min, y_max); //5.5 works
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min, z_max); //5.5 works
  pass.filter (*cloud_filtered);

  //std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
//  seg.setOptimizeCoefficients (true);
//  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
//  //seg.setNormalDistanceWeight (0.1);
//  seg.setMethodType (pcl::SAC_RANSAC);
//  //seg.setMaxIterations (100);
//  seg.setDistanceThreshold (0.01); //0.31 if 1 segmentation
//  seg.setInputCloud (cloud_filtered);
//  seg.setInputNormals (cloud_normals);
//  // Obtain the plane inliers and coefficients
//  seg.segment (*inliers_plane, *coefficients_plane);
//  //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

//  // Extract the planar inliers from the input cloud
//  extract.setInputCloud (cloud_filtered);
//  extract.setIndices (inliers_plane);
//  extract.setNegative (false);

//  // Write the planar inliers to disk
//  pcl::PointCloud<PointT> cloud_plane; //(new pcl::PointCloud<PointT> ());
//  extract.filter (cloud_plane);
//  //std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//  //writer.write ("src/P26_cylinder_segmentation/pointclouds/plane.pcd", *cloud_plane, false);

//  // Remove the planar inliers, extract the rest
//  extract.setNegative (true);
//  extract.filter (*cloud_filtered2);
//  extract_normals.setNegative (true);
//  extract_normals.setInputCloud (cloud_normals);
//  extract_normals.setIndices (inliers_plane);
//  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.2); //0.01 works (0.1 seperated the robot from the cylinder)
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.08);//0.1 works
  seg.setRadiusLimits (cylinderRadius-0.03, cylinderRadius+0.03);//0.08, 0.16 works
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);


  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()){
//  Can't find the cylindrical component.
    std::cerr << "x";
  }

//    writer.write ("input_cloud.pcd", *input, false);
//    writer.write ("cloud_filtered.pcd", *cloud_filtered, false);
//    writer.write ("cylinder.pcd", *cloud_cylinder, false);


  return cloud_cylinder;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr
passThroughFilterSphere(pcl::PointCloud<PointT>::Ptr& cloud,
pcl::PointXYZ sphereCenterPoint, const double radius, bool remove_outside)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
  float distanceFromSphereCenterPoint;
  bool pointIsWithinSphere;
  bool addPointToFilteredCloud;
  for (int point_i = 0; point_i < cloud->size(); ++point_i)
  {
    distanceFromSphereCenterPoint = pcl::euclideanDistance(cloud->at(point_i), sphereCenterPoint);
    pointIsWithinSphere = distanceFromSphereCenterPoint <= radius;
    addPointToFilteredCloud = (!pointIsWithinSphere && remove_outside) || (pointIsWithinSphere && !remove_outside);
    if (addPointToFilteredCloud){
      filteredCloud->push_back(cloud->at(point_i));
    }
  }
  return filteredCloud;
}
