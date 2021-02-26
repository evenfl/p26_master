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


  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "p26_lefty_tcp";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
    //ros::shutdown();


    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 5;
    target_pose1.position.y = 5;
    target_pose1.position.z = 1;
    move_group.setPoseTarget(target_pose1);



    moveit::core::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;

    start_state.setFromIK(joint_model_group, start_pose2);
    move_group.setStartState(start_state);

    // Now we will plan to the earlier pose target from the new
    // start state that we have just created.
    move_group.setPoseTarget(target_pose1);

    // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
    // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
    move_group.setPlanningTime(10.0);

    std::cerr << (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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
