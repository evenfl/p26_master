cmake_minimum_required(VERSION 3.0.2)
project(p26_pick_and_place)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


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



  interactive_markers
  pluginlib
  geometric_shapes
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

set(THIS_PACKAGE_INCLUDE_DIRS
  include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cylinder_segmentation
#  CATKIN_DEPENDS pcl roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
    LIBRARIES
      interactivity_utils
    INCLUDE_DIRS
      ${THIS_PACKAGE_INCLUDE_DIRS}
    CATKIN_DEPENDS
      pcl_ros
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
    DEPENDS
      EIGEN3
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

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(collision_map src/main.cpp)

target_link_libraries (collision_map ${catkin_LIBRARIES})


add_executable(merge_clouds src/merge_clouds.cpp)

target_link_libraries (merge_clouds ${catkin_LIBRARIES})

#add_executable(main_cpp src/main.cpp)

#target_link_libraries (main_cpp ${catkin_LIBRARIES})#, ${PCL_LIBRARIES})
# target_link_libraries(cylinder_seg [${catkin_LIBRARIES} [${PCL_LIBRARIES}]])
# add_dependencies(cylinder_seg cyl_seg_cpp)
