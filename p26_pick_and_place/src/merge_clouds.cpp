#include "main.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

PointCloud::Ptr cloud_merged (new PointCloud);
void callback1(const sensor_msgs::PointCloud2ConstPtr& input);
void callback2(const sensor_msgs::PointCloud2ConstPtr& input);
void callback3(const sensor_msgs::PointCloud2ConstPtr& input);
void callback4(const sensor_msgs::PointCloud2ConstPtr& input);
void callback5(const sensor_msgs::PointCloud2ConstPtr& input);
void callback6(const sensor_msgs::PointCloud2ConstPtr& input);
int counter = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_map");
  ros::NodeHandle nh;
//  ros::Subscriber sub1 = nh.subscribe ("/master/jetson1/kinect_decomp", 1, callback);
//  ros::Subscriber sub2 = nh.subscribe ("/master/jetson2/kinect_decomp", 1, callback);
//  ros::Subscriber sub3 = nh.subscribe ("/master/jetson3/kinect_decomp", 1, callback);
//  ros::Subscriber sub4 = nh.subscribe ("/master/jetson4/kinect_decomp", 1, callback);
//  ros::Subscriber sub5 = nh.subscribe ("/master/jetson5/kinect_decomp", 1, callback);
//  ros::Subscriber sub6 = nh.subscribe ("/master/jetson6/kinect_decomp", 1, callback);
  ros::Subscriber sub1 = nh.subscribe ("/p26_lefty/jetson1/wp3/kinect_decomp", 1, callback1);
  ros::Subscriber sub2 = nh.subscribe ("/p26_lefty/jetson2/wp3/kinect_decomp", 1, callback2);
  ros::Subscriber sub3 = nh.subscribe ("/p26_lefty/jetson3/wp3/kinect_decomp", 1, callback3);
  ros::Subscriber sub4 = nh.subscribe ("/p26_lefty/jetson4/wp3/kinect_decomp", 1, callback4);
  ros::Subscriber sub5 = nh.subscribe ("/p26_lefty/jetson5/wp3/kinect_decomp", 1, callback5);
  ros::Subscriber sub6 = nh.subscribe ("/p26_lefty/jetson6/wp3/kinect_decomp", 1, callback6);
//  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/master/points_nocolor_merged", 1);
  ros::Publisher pub = nh.advertise<PointCloud> ("/p26_lefty/kinect_decomp_merged", 1);
  //ros::Publisher collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("p26_lefty/collision_object", 1);

  ros::Rate rate(20);
  cloud_merged->header.frame_id = "world";
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    if (counter >= 5)
    {

      std::cerr << cloud_merged->size() << std::endl;
      cloud_merged->header.frame_id = "world";
      pub.publish(cloud_merged);
      cloud_merged->clear();
      counter = 0;

    }

  }

}


void callback1(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}

void callback2(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
void callback3(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
void callback4(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
void callback5(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
void callback6(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
