
#include "main.h"
#include "segment.h"
#include "addCylinder.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::Ptr cloud_merged (new PointCloud);
PointCloud::Ptr cloud_cylinder (new PointCloud);
PointCloud::Ptr cloud_cylinder_tmp (new PointCloud);
void callback(const sensor_msgs::PointCloud2ConstPtr& input);
int counter = 0;

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
//  ros::Publisher pub = nh.advertise<PointCloud> ("cloud_cylinder", 1);
  //ros::Publisher cylinder_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  //ros::Publisher pub_com = nh.advertise<geometry_msgs::Point> ("cylinder_com", 1);
  //ros::Publisher pub_dirvec = nh.advertise<geometry_msgs::Point> ("cylinder_dirvec", 1);

  ros::Rate rate(20);

  int iterations = 0;

  PointT point_com_avg;
  PointT dirvec_avg;
  int nrOfIterations = 20;




  //For testing results
  nrOfIterations = 20;
  int nrOfSamples = 0;
  PointT correctcom;
  correctcom.x = 5.2751022105+0.022;
  correctcom.y = 5.2885478721+0.0226;
  correctcom.z = 0.0022119397+0.6+0.0574;//+0.155


  //correctcom.x = 5.2751022105;
  //correctcom.y = 5.2885478721;
  //correctcom.z = 0.0022119397+0.6+0.155;

  PointT correctdirvec;
  correctdirvec.x = 0;
  correctdirvec.y = 0;
  correctdirvec.z = 1;
  double avg_com_error = 0.0;
  double avg_dirvec_error = 0.0;
  int maxNrOfSamples = 10000;


  const float deviance = 0.06;

  unsigned int inconsistencyCounter = 0;

  std::cerr << "Segmenting cylinder." << std::endl;

  std::cerr << "0%";
  for (int i = 0; i < nrOfIterations-6; i++){std::cerr << " ";};
  std::cerr << "100%" << std::endl;



  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    if (counter >= 6)
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
//        cloud_cylinder_tmp = passThroughFilterSphere(cloud_cylinder, point, cylinderLength/2, false);
        cloud_cylinder_tmp = passThroughFilterSphere(cloud_cylinder, point, sqrt((cylinderLength/2)*(cylinderLength/2)+cylinderRadius*cylinderRadius), false);
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
        //std::cerr << "#";
      }

      if ( std::abs(dirvec.x - dirvec_avg.x) > 0.5 || std::abs(dirvec.y - dirvec_avg.y) > 0.5 || std::abs(dirvec.z - dirvec_avg.z) > 0.5  )
      {
        dirvec.x = -dirvec.x;
        dirvec.y = -dirvec.y;
        dirvec.z = -dirvec.z;
      }


      if ( std::abs(point_com.x - point_com_avg.x) > deviance || std::abs(point_com.z - point_com_avg.z) > deviance || std::abs(point_com.z - point_com_avg.z) > deviance  )
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
        //std::cerr << "#";
      }
      if (iterations >= nrOfIterations)
      {
        //cylinder_object_publisher.publish(addCylinder(cylinder_params));
        //UNCOMMENT THIS:
        //std::cerr << std::endl << "Direction vector: [ " << dirvec_avg.x << ", " << dirvec_avg.y << ", " << dirvec_avg.z << " ]" << std::endl;
        //std::cerr << std::endl << "Centre of mass:   [ " << point_com_avg.x << ", " << point_com_avg.y << ", " << point_com_avg.z << " ]" << std::endl << std::endl;


        cylinder_params.center_pt[0] = point_com_avg.x;
        cylinder_params.center_pt[1] = point_com_avg.y;
        cylinder_params.center_pt[2] = point_com_avg.z;

        cylinder_params.height = cylinderLength*1.1;
        cylinder_params.radius = cylinderRadius*1.1;

        cylinder_params.direction_vec[0] = dirvec_avg.x;
        cylinder_params.direction_vec[1] = dirvec_avg.y;
        cylinder_params.direction_vec[2] = dirvec_avg.z;
        //cylinder_object_publisher.publish(addCylinder(cylinder_params));
        geometry_msgs::Point pub_com_msg;
        geometry_msgs::Point pub_dirvec_msg;
        pub_com_msg.x = point_com_avg.x;
        pub_com_msg.y = point_com_avg.y;
        pub_com_msg.z = point_com_avg.z;
        pub_dirvec_msg.x = dirvec_avg.x;
        pub_dirvec_msg.y = dirvec_avg.y;
        pub_dirvec_msg.z = dirvec_avg.z;
        //pub_com.publish(pub_com_msg);
        //pub_dirvec.publish(pub_dirvec_msg);

//        pcl::PCDWriter writer;
//        writer.write ("src/p26_master/p26_cylinder_segmentation/pointclouds/cylinder_filtered.pcd", *cloud_cylinder, false);
//        writer.write ("src/p26_master/p26_cylinder_segmentation/pointclouds/cloud_merged.pcd", *cloud_merged, false);


                //For testing
        nrOfSamples++;
        std::cerr << "#";
        double xce = point_com_avg.x - correctcom.x;
        double yce = point_com_avg.y - correctcom.y;
        double zce = point_com_avg.z - correctcom.z;
        double xde = dirvec_avg.x-correctdirvec.x;
        double yde = dirvec_avg.y-correctdirvec.y;
        double zde = std::abs(dirvec_avg.z)-std::abs(correctdirvec.z);
        double error_com = sqrt(xce*xce + yce*yce + zce*zce);
        double error_dirvec = sqrt(xde*xde + yde*yde + zde*zde);
        avg_com_error = (avg_com_error*(nrOfSamples-1)+sqrt(xce*xce + yce*yce + zce*zce))/nrOfSamples;
        avg_dirvec_error = (avg_dirvec_error*(nrOfSamples-1)+sqrt(xde*xde + yde*yde + zde*zde))/nrOfSamples;

        if (nrOfIterations != 100)
        {
          std::string filename = "result_validation";
          filename += std::to_string(nrOfIterations);
          filename += ".txt";
          std::ofstream myfile;
          myfile.open(filename, std::ios_base::app);
          if (myfile.is_open()){
            myfile << point_com_avg.x << ", " << point_com_avg.y << ", " << point_com_avg.z << ", " << dirvec_avg.x << ", " << dirvec_avg.y << ", " << dirvec_avg.z << "\n";
          }
        }

        if (nrOfIterations == 10000){

          nrOfIterations = 1;
          maxNrOfSamples = 1000;
          correctcom.x = point_com_avg.x;
          correctcom.y = point_com_avg.y;
          correctcom.z = point_com_avg.z;
          correctdirvec.x = dirvec_avg.x;
          correctdirvec.y = dirvec_avg.y;
          correctdirvec.z = dirvec_avg.z;
          std::cerr << std::endl << "Direction vector: [ " << dirvec_avg.x << ", " << dirvec_avg.y << ", " << dirvec_avg.z << " ]" << std::endl;
          std::cerr << std::endl << "Centre of mass:   [ " << point_com_avg.x << ", " << point_com_avg.y << ", " << point_com_avg.z << " ]" << std::endl << std::endl;
          nrOfSamples = 0;
          avg_com_error = 0.0;
          avg_dirvec_error = 0.0;
        }


        point_com_avg.x = 0.0;
        point_com_avg.y = 0.0;
        point_com_avg.z = 0.0;
        dirvec_avg.x = 0.0;
        dirvec_avg.y = 0.0;
        dirvec_avg.z = 0.0;
        iterations = 0;


//        std::cerr << "com: " << xce << ", "<< yce << ", "<< zce << ", dir: " << xde << ", "<< yde << ", "<< zde << std::endl;
        if(nrOfSamples >=maxNrOfSamples){
            std::cerr << std::endl;
            std::cerr << nrOfIterations << ", " << avg_com_error << std::endl;
            std::cerr << nrOfIterations << ", " << avg_dirvec_error << std::endl;
            std::cerr << std::endl;
            nrOfSamples = 0;
            avg_com_error = 0.0;
            avg_dirvec_error = 0.0;
            nrOfIterations++;
            if(nrOfIterations>1){
              ros::shutdown();
            }
        }





      }
        cloud_merged->clear();
    }

  }

}


void callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<PointT> cloud;
  pcl::fromROSMsg (*input, cloud);//cloud is the output
  *cloud_merged += cloud;
  counter++;
}

