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

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = "world";
      collision_object.id = "box";
      int nrOfBoxes = 0;
      int nrOfPoints = cloud_merged->size();
      double x[nrOfPoints];
      double y[nrOfPoints];
      double z[nrOfPoints];
      std::cerr << "Number of points: " << nrOfPoints << std::endl;
      //for (int point_i = 0; point_i < 5; ++point_i)//
      for (int point_i = 0; point_i < cloud_merged->size(); ++point_i)
      {

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        primitive.dimensions[0] = 0.12;
        primitive.dimensions[1] = primitive.dimensions[0];
        primitive.dimensions[2] = primitive.dimensions[0];

        geometry_msgs::Pose box_pose;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;
        box_pose.position.x = cloud_merged->at(point_i).x;
        box_pose.position.y = cloud_merged->at(point_i).y;
        box_pose.position.z = cloud_merged->at(point_i).z;

        for (int i = 0; i < point_i; i++){
          if (x[i] == box_pose.position.x && y[i] == box_pose.position.y && z[i] == box_pose.position.z){
            //std::cerr << "broke loop at " << i << std::endl;
            break;
          }
          else if (i == point_i-1){
            x[point_i] = box_pose.position.x;
            y[point_i] = box_pose.position.y;
            z[point_i] = box_pose.position.z;

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            nrOfBoxes++;
          }

          //std::cerr << i << ", " << point_i << std::endl;
        }

      }

      //collision_object_publisher.publish(collision_object);


      std::cerr << "Number of boxes: " << nrOfBoxes << std::endl;

      counter = 0;

      ros::shutdown();
      cloud_merged->clear();

    }

  }

}


void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  //cloud_merged->clear();
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}
