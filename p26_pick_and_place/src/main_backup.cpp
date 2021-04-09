#include "main.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr cloud_merged (new PointCloud);
void callback(const sensor_msgs::PointCloud2ConstPtr& input);
int counter = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_map");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud>("/jetson/wp3/points_nocolor_merged", 1);
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
    if (counter >= 5)
    {
      cloud_merged->header.frame_id = "world";

      pub.publish(*cloud_merged);

      ros::shutdown();

    }

  }

}


void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //cloud_merged->clear();
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
