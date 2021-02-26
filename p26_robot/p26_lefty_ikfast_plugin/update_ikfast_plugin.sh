search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=p26_lefty.srdf
robot_name_in_srdf=p26_lefty
moveit_config_pkg=p26_lefty_moveit_config
robot_name=p26_lefty
planning_group_name=p26_lefty_tcp
ikfast_plugin_pkg=p26_lefty_ikfast_plugin
base_link_name=lefty_track_left
eef_link_name=lefty_tool
ikfast_output_path=/home/sfi/catkin_ws/src/p26_master/p26_robot/p26_lefty_ikfast_plugin/src/p26_lefty_p26_lefty_tcp_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
