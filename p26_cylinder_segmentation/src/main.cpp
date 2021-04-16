#include "main.h"
#include "segment.h"
#include "addCylinder.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr cloud_merged (new PointCloud);
PointCloud::Ptr cloud_cylinder (new PointCloud);
PointCloud::Ptr cloud_cylinder_tmp (new PointCloud);
void callback(const sensor_msgs::PointCloud2ConstPtr& input);
void callback_find_cylinder(const std_msgs::String::ConstPtr& data);
int counter = 0;
bool findCylinder = true;

// Declare a variable of type AddCylinderParams and store relevant values from ModelCoefficients.
AddCylinderParams cylinder_params;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub1 = nh.subscribe ("/master/jetson1/kinect_decomp", 1, callback);
  ros::Subscriber sub2 = nh.subscribe ("/master/jetson2/kinect_decomp", 1, callback);
  ros::Subscriber sub3 = nh.subscribe ("/master/jetson3/kinect_decomp", 1, callback);
  ros::Subscriber sub4 = nh.subscribe ("/master/jetson4/kinect_decomp", 1, callback);
  ros::Subscriber sub5 = nh.subscribe ("/master/jetson5/kinect_decomp", 1, callback);
  ros::Subscriber sub6 = nh.subscribe ("/master/jetson6/kinect_decomp", 1, callback);
  ros::Subscriber sub_find_cylinder = nh.subscribe ("/p26_lefty/find_cylinder", 1, callback_find_cylinder);
//  ros::Subscriber sub6 = nh.subscribe ("/master/merged_point_cloud", 1, callback);
  ros::Publisher pub = nh.advertise<PointCloud> ("cloud_cylinder", 1);
  ros::Publisher cylinder_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  ros::Publisher pub_com = nh.advertise<geometry_msgs::Point> ("cylinder_com", 1);
  ros::Publisher pub_dirvec = nh.advertise<geometry_msgs::Point> ("cylinder_dirvec", 1);

  ros::Rate rate(20);

  int iterations = 0;

  PointT point_com_avg;
  PointT dirvec_avg;
  const int nrOfIterations = 20;
//  const int nrOfIterations = 2;
  const float deviance = 0.06; // For 26 cm cylinder
//  const float deviance = 0.1; // For 16 cm cylinder

  unsigned int inconsistencyCounter = 0;

  std::cerr << "Segmenting cylinder." << std::endl;

  std::cerr << "0%";
  for (int i = 0; i < nrOfIterations-6; i++){std::cerr << " ";};
  std::cerr << "100%" << std::endl;



  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (counter >= 6 && findCylinder == true)
    {
      // If all pointclouds are received, find the pose

      pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
      cloud_cylinder = segment(cloud_merged, coefficients_cylinder);

      PointT point;
      point.x = coefficients_cylinder->values[0];
      point.y = coefficients_cylinder->values[1];
      point.z = coefficients_cylinder->values[2];
      PointT dirvec;
      dirvec.x = coefficients_cylinder->values[3];
      dirvec.y = coefficients_cylinder->values[4];
      dirvec.z = coefficients_cylinder->values[5];

      float adj = 0.0f;
      float increment = 0.001f; //Check again every 1 mm

      adj = (x_min - point.x)/dirvec.x;
      point.x = x_min;
      point.y = point.y + adj*dirvec.y;
      point.z = point.z + adj*dirvec.z;

      if (point.y < y_min)
      {
        adj = (y_min - point.y)/dirvec.y;
        point.x = point.x + adj*dirvec.x;
        point.y = y_min;
        point.z = point.z + adj*dirvec.z;
      }
      else if (point.y > y_max)
      {
        adj = (y_max - point.y)/dirvec.y;
        point.x = point.x + adj*dirvec.x;
        point.y = y_max;
        point.z = point.z + adj*dirvec.z;
      }
      if (point.z < z_min)
      {
        adj = (z_min - point.z)/dirvec.z;
        point.x = point.x + adj*dirvec.x;
        point.y = point.y + adj*dirvec.y;
        point.z = z_min;
      }
      else if (point.z > z_max)
      {
        adj = (z_max - point.z)/dirvec.z;
        point.x = point.x + adj*dirvec.x;
        point.y = point.y + adj*dirvec.y;
        point.z = z_max;
      }
      if (point.x + increment*dirvec.x > x_max || point.x + increment*dirvec.x < x_min ||
          point.y + increment*dirvec.y > y_max || point.y + increment*dirvec.y < y_min ||
          point.z + increment*dirvec.z > z_max || point.z + increment*dirvec.z < z_min)
          {
            increment = std::abs(increment)*(-1);
          }

      PointT point_com;
      unsigned long biggestCloudSize = 0;

      while (point.x >= x_min && point.x <= x_max && point.y >= y_min &&  point.y <= y_max && point.z >= z_min && point.z <= z_max)
      {
        cloud_cylinder_tmp = passThroughFilterSphere(cloud_cylinder, point, cylinderLength/2, false);
        if (cloud_cylinder_tmp->size () > biggestCloudSize)
        {
          biggestCloudSize = cloud_cylinder_tmp->size ();
          point_com = point;
        }
        point.x = point.x + increment*dirvec.x;
        point.y = point.y + increment*dirvec.y;
        point.z = point.z + increment*dirvec.z;
      }


      cloud_cylinder = passThroughFilterSphere(cloud_cylinder, point_com, sqrt((cylinderLength/2)*(cylinderLength/2)+cylinderRadius*cylinderRadius), false);

      counter = 0;

      if (iterations == 0 && point_com.x != 0.0)
      {
        point_com_avg.x = point_com.x;
        point_com_avg.y = point_com.y;
        point_com_avg.z = point_com.z;
        dirvec_avg.x = dirvec.x;
        dirvec_avg.y = dirvec.y;
        dirvec_avg.z = dirvec.z;
        iterations++;
        inconsistencyCounter = 0;
        std::cerr << "#";
      }

      if ( std::abs(dirvec.x - dirvec_avg.x) > 0.5 || std::abs(dirvec.y - dirvec_avg.y) > 0.5 || std::abs(dirvec.z - dirvec_avg.z) > 0.5  )
      {
        dirvec.x = -dirvec.x;
        dirvec.y = -dirvec.y;
        dirvec.z = -dirvec.z;
      }


      if ( std::abs(point_com.x - point_com_avg.x) > deviance || std::abs(point_com.z - point_com_avg.z) > deviance || std::abs(point_com.z - point_com_avg.z) > deviance && point_com.x != 0 )
      {
        inconsistencyCounter++;
        std::cerr << "-";
        if (inconsistencyCounter >= round(0.25*nrOfIterations))
        {
          iterations = 0;
          inconsistencyCounter = 0;
          std::cerr << " Inconsistent result, starting over." << std::endl;
        }
      }
      else if (point_com.x != 0.0)
      {
        point_com_avg.x = (point_com_avg.x*iterations + point_com.x)/(iterations+1);
        point_com_avg.y = (point_com_avg.y*iterations + point_com.y)/(iterations+1);
        point_com_avg.z = (point_com_avg.z*iterations + point_com.z)/(iterations+1);
        dirvec_avg.x = (dirvec_avg.x*iterations + dirvec.x)/(iterations+1);
        dirvec_avg.y = (dirvec_avg.y*iterations + dirvec.y)/(iterations+1);
        dirvec_avg.z = (dirvec_avg.z*iterations + dirvec.z)/(iterations+1);
        iterations++;
        //inconsistencyCounter = 0;
        std::cerr << "#" << iterations;
      }
      if (iterations >= nrOfIterations)
      {
        //cylinder_object_publisher.publish(addCylinder(cylinder_params));
        std::cerr << std::endl << "Direction vector: [ " << dirvec_avg.x << ", " << dirvec_avg.y << ", " << dirvec_avg.z << " ]" << std::endl;
        std::cerr << std::endl << "Centre of mass:   [ " << point_com_avg.x << ", " << point_com_avg.y << ", " << point_com_avg.z << " ]" << std::endl << std::endl;
        iterations = 0;

        cylinder_params.center_pt[0] = point_com_avg.x;
        cylinder_params.center_pt[1] = point_com_avg.y;
        cylinder_params.center_pt[2] = point_com_avg.z;

        cylinder_params.height = cylinderLength;
        cylinder_params.radius = cylinderRadius;

        cylinder_params.direction_vec[0] = dirvec_avg.x;
        cylinder_params.direction_vec[1] = dirvec_avg.y;
        cylinder_params.direction_vec[2] = dirvec_avg.z;
        cylinder_object_publisher.publish(addCylinder(cylinder_params));
        geometry_msgs::Point pub_com_msg;
        geometry_msgs::Point pub_dirvec_msg;
        pub_com_msg.x = point_com_avg.x;
        pub_com_msg.y = point_com_avg.y;
        pub_com_msg.z = point_com_avg.z;
        pub_dirvec_msg.x = dirvec_avg.x;
        pub_dirvec_msg.y = dirvec_avg.y;
        pub_dirvec_msg.z = dirvec_avg.z;
        pub_com.publish(pub_com_msg);
        pub_dirvec.publish(pub_dirvec_msg);


        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        collision_object.id = "box";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.3;
        primitive.dimensions[1] = 0.4;
        primitive.dimensions[2] = 0.25;
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 5.42;
        box_pose.position.y = 9.23;
        box_pose.position.z = 0.125;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        cylinder_object_publisher.publish(collision_object);

//        pcl::PCDWriter writer;
//        writer.write ("src/p26_master/p26_cylinder_segmentation/pointclouds/cylinder_filtered.pcd", *cloud_cylinder, false);
//        writer.write ("src/p26_master/p26_cylinder_segmentation/pointclouds/cloud_merged.pcd", *cloud_merged, false);

        findCylinder = false;
//        ros::shutdown();
      }

      cloud_merged->header.frame_id = "world";
      pub.publish(*cloud_merged);
      cloud_merged->clear();
    }
    else if(findCylinder == false){
      cloud_merged->clear();
      counter = 0;
    }

  }

}


void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
//  cloud_merged->clear();
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}


void callback_find_cylinder(const std_msgs::String::ConstPtr& data)
{
  findCylinder = true;
}
