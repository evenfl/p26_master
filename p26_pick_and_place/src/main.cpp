#include "main.h"

#include <std_srvs/Empty.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//PointCloud::Ptr cloud_merged (new PointCloud);


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2ConstPtr& input);
void callback_create_octomap(const std_msgs::String::ConstPtr& msg);
int counter = 0;
bool createOctomap = false;
//bool createOctomap = true;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_map");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud>("/master/merged_point_cloud", 1);
  ros::Publisher pub_octomap_created = nh.advertise<std_msgs::String>("/p26_lefty/octomap_created", 1);
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

  ros::Subscriber sub_create_octomap = nh.subscribe ("/p26_lefty/create_octomap", 1, callback_create_octomap);

//  ros::Publisher pub = nh.advertise<PointCloud> ("/jetson/wp3/points_nocolor", 1);

  ros::Rate rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (counter >= 12 && createOctomap == true)
    {
      cloud_merged->header.frame_id = "world";


      // Create the filtering object
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_merged);
      sor.setMeanK (50);
      sor.setStddevMulThresh (0.25);
      sor.filter (*cloud_filtered);


//      pcl::PCDWriter writer;
//      writer.write ("cloud_unfiltered.pcd", *cloud_merged, false);
//      writer.write ("cloud_filtered_sor.pcd", *cloud_filtered, false);


      //ros::service::waitForService("p26_lefty/clear_octomap");  //this is optional
      ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("p26_lefty/clear_octomap");
      std_srvs::Empty srv;
      clearClient.call(srv);
      pub.publish(*cloud_filtered);

//      std::cerr << cloud_merged->size() << std::endl;

      cloud_merged->clear();
      cloud_filtered->clear();
      counter = 0;
      createOctomap = false;
      std_msgs::String msg;
      std::stringstream ss;
      int count = 0;
      ss << "1" << count;
      msg.data = ss.str();
      pub_octomap_created.publish(msg);
//      ros::shutdown();

    }
    else if(createOctomap == false){
      cloud_merged->clear();
      counter = 0;
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

void callback_create_octomap(const std_msgs::String::ConstPtr& msg)
{
  createOctomap = true;
}
