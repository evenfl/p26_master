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

  ros::Publisher planning_scene_plan_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("p26_lefty/move_group/display_planned_path", 1);

  //Load robot model with robot_model_loader
  robot_model_loader::RobotModelLoader robot_model_loader("p26_lefty/robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());

  planning_scene::PlanningScenePtr planning_scene_plan(new planning_scene::PlanningScene(robot_model));

  //  Get robot kinematic state for lefty_tool
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(robot_model));
  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("lefty_tool0");

  //  Printing robot cordinates and angles for lefty_tool
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");


  //  Make a new planning pipeline
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
    new planning_pipeline::PlanningPipeline(robot_model, nh, "planning_plugin", "request_adapters"));


  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;

  // Define position and end effector to utilize
  pose.header.frame_id = "lefty_tool";
  pose.pose.position.x = 5;//point_com.x;
  pose.pose.position.y = 6;//point_com.y;
  pose.pose.position.z = 1;//point_com.z;
  pose.pose.orientation.w = 1.0;

  // Define tolerance for pose position and angle
  std::vector<double> tolerance_pose(3, 0.2);
  std::vector<double> tolerance_angle(3, 0.01);

  req.group_name = "p26_lefty";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("lefty_tool", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Now, call the pipeline and check whether planning was successful.
  planning_pipeline->generatePlan(planning_scene_plan, req, res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  else
  {
    ROS_INFO("Plan computed successfully, executing in 5s");
  }

  ros::Duration(5).sleep();




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
