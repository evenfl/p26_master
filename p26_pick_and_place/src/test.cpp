#include "main.h"

geometry_msgs::Point point_com;
geometry_msgs::Point dirvec;

void callback_com(const geometry_msgs::Point point);
void callback_dirvec(const geometry_msgs::Point vec);

int main(int argc, char **argv) {
  ros::init(argc, argv, "p26_pick_and_place");
  ros::NodeHandle nh;
  ros::Rate rate(20);

  ros::Subscriber sub_com = nh.subscribe ("cylinder_com", 1, callback_com);
  ros::Subscriber sub_dirvec = nh.subscribe ("cylinder_dirvec", 1, callback_dirvec);



  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("lefty");
  group.setPlanningTime(45.0);


  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    //ros::shutdown();
    ros::WallDuration(1.0).sleep();


    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "lefty_joint_6";
    tf2::Quaternion orientation;
    orientation.setRPY(0,0,1);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 5.0;
    grasps[0].grasp_pose.pose.position.y = 5.0;
    grasps[0].grasp_pose.pose.position.z = 1.0;



    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "lefty_joint_6";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

  }

  return 0;
}


void callback_com(geometry_msgs::Point point)
{
  point_com = point;
  std::cerr << std::endl << "Centre of mass:   [ " << point_com.x << ", " << point_com.y << ", " << point_com.z << " ]" << std::endl << std::endl;
}

void callback_dirvec(geometry_msgs::Point vec)
{
  dirvec = vec;
  std::cerr << std::endl << "Direction vector: [ " << dirvec.x << ", " << dirvec.y << ", " << dirvec.z << " ]" << std::endl;
}
