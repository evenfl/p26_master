#include "addCylinder.h"

//moveit_msgs::PlanningScene addCylinder(const AddCylinderParams cylinder_params)

moveit_msgs::CollisionObject addCylinder(const AddCylinderParams cylinder_params)
  {
    moveit_msgs::PlanningScene planning_scene_interface;
    // BEGIN_SUB_TUTORIAL add_cylinder
    //
    // Adding Cylinder to Planning Scene
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = "cylinder";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    /* Setting height of cylinder. */
    primitive.dimensions[0] = cylinder_params.height;
    /* Setting radius of cylinder. */
    primitive.dimensions[1] = cylinder_params.radius;

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(cylinder_params.direction_vec[0], cylinder_params.direction_vec[1],
                                         cylinder_params.direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    cylinder_pose.position.x = cylinder_params.center_pt[0];
    cylinder_pose.position.y = cylinder_params.center_pt[1];
    cylinder_pose.position.z = cylinder_params.center_pt[2];

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;
    //planning_scene_interface.addCollisionObject(collision_object);
    //planning_scene_interface.world.collision_objects.push_back(collision_object);
    // END_SUB_TUTORIAL

    //return planning_scene_interface;
    return collision_object;

  }
