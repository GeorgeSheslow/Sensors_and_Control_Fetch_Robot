#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/subscriber.h>

#include "fetch_robot_sim/Location_3D.h"
#include "iksolver/Location_3D.h"

// Global Parameters
float x = 0.0;
float y = 0.0;
float z = 0.0;
moveit::planning_interface::MoveGroupInterface move_group_interface("arm");
const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup("arm");

bool calcTraj(fetch_robot_sim::Location_3DConstPtr &req,
          moveit_msgs::RobotTrajectory &res){

  ROS_INFO("Calculate Trajectory Request Received.");

  /* ********
  // Account for base movements
  // const Eigen::Affine3d &sensor_state = move_group_interface.getCurrentState()->getGlobalLinkTransform("head_tilt_link");
  
  // Print end-effector pose
  // ROS_INFO_STREAM("Translation: " << sensor_state.translation());
  // ROS_INFO_STREAM("Rotation: " << sensor_state.rotation());
  ******** */

  // Extract request from sensor
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = req->x;
  target_pose1.position.y = req->y;
  target_pose1.position.z = req->z;
  move_group_interface.setPoseTarget(target_pose1);

  // calculate trajectory
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  if( not (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) ){
    return false;
  }

  // Update robot model to reflect new state
  move_group_interface.execute(my_plan);

  // Return calculated trajectory
  res = my_plan.trajectory_;

  return true;

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "iksolver_ex");
  ros::NodeHandle node_handle;

  // service 
  ros::ServiceServer service = node_handle.advertiseService("calc_traj", calcTraj);

  ros::spin();
  return 0;
}