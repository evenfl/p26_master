cmake_minimum_required(VERSION 3.0.2)
project(p26_pick_and_place)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS pcl_ros
#  INCLUDE_DIRS include
#  LIBRARIES cylinder_segmentation
#  CATKIN_DEPENDS pcl roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(main_cpp src/main.cpp)

target_link_libraries (main_cpp ${catkin_LIBRARIES})#, ${PCL_LIBRARIES})
# target_link_libraries(cylinder_seg [${catkin_LIBRARIES} [${PCL_LIBRARIES}]])
# add_dependencies(cylinder_seg cyl_seg_cpp)
