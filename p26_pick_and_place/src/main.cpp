#include "main.h"

#include <std_srvs/Empty.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//PointCloud::Ptr cloud_merged (new PointCloud);


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2ConstPtr& input);
int counter = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_map");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud>("/master/merged_point_cloud", 1);
//  ros::Subscriber sub1 = nh.subscribe ("/p26_lefty/jetson1/wp3/kinect_decomp", 1, callback);
//  ros::Subscriber sub2 = nh.subscribe ("/p26_lefty/jetson2/wp3/kinect_decomp", 1, callback);
//  ros::Subscriber sub3 = nh.subscribe ("/p26_lefty/jetson3/wp3/kinect_decomp", 1, callback);
//  ros::Subscriber sub4 = nh.subscribe ("/p26_lefty/jetson4/wp3/kinect_decomp", 1, callback);
//  ros::Subscriber sub5 = nh.subscribe ("/p26_lefty/jetson5/wp3/kinect_decomp", 1, callback);
//  ros::Subscriber sub6 = nh.subscribe ("/p26_lefty/jetson6/wp3/kinect_decomp", 1, callback);

  ros::Subscriber sub1 = nh.subscribe ("/master/jetson1/kinect_decomp", 1, callback);
  ros::Subscriber sub2 = nh.subscribe ("/master/jetson2/kinect_decomp", 1, callback);
  ros::Subscriber sub3 = nh.subscribe ("/master/jetson3/kinect_decomp", 1, callback);
  ros::Subscriber sub4 = nh.subscribe ("/master/jetson4/kinect_decomp", 1, callback);
  ros::Subscriber sub5 = nh.subscribe ("/master/jetson5/kinect_decomp", 1, callback);
  ros::Subscriber sub6 = nh.subscribe ("/master/jetson6/kinect_decomp", 1, callback);

//  ros::Publisher pub = nh.advertise<PointCloud> ("/jetson/wp3/points_nocolor", 1);
  ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("p26_lefty/collision_object", 1);

  ros::Rate rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    if (counter >= 6)
    {
      cloud_merged->header.frame_id = "world";

      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_merged);
      sor.setMeanK (50);
      sor.setStddevMulThresh (0.25);
      sor.filter (*cloud_filtered);

      ros::service::waitForService("p26_lefty/clear_octomap");  //this is optional
      ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("p26_lefty/clear_octomap");
      std_srvs::Empty srv;
      clearClient.call(srv);

      pub.publish(*cloud_filtered);

      cloud_merged->clear();
      cloud_filtered->clear();
      counter = 0;
//      ros::shutdown();

    }

  }

}


void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //cloud_merged->clear();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
