#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

#include "fetch_robot_sim/IkInfo.srv"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

bool get_ik(fetch_robot_sim::IkInfo::Request &req,
            fetch_robot_sim::IkInfo::Response &res) {
  geometry_msgs::PoseStamped pose;
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  pose.pose.position.x = req.x;
  pose.pose.position.y = req.y;
  pose.pose.position.z = req.z;

  tf2::Quaternion quat_tf;
  quat_tf.setRPY(req.r, req.p, req.y);
  quat_tf.normalize();
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);
  // Does not change, always pointing 90 degrees
  pose.quarterion = quat_msg;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints(
          "wrist_roll_link", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req,
                                           res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  robot_state->setJointGroupPositions(
      joint_model_group,
      response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // PRINT OUT JOINT VALUES
  const std::vector<std::string> &joint_names =
      joint_model_group->getVariableNames();
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // setup return message

  return true;
}

int main(int argc, char **argv) {
  const std::string node_name = "fetch_iksolver";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle;

  ros::ServicewServer service = n.advertiseService("ik_info", get_ik);
  const std::string PLANNING_GROUP = "arm";
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &robot_model =
      robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot
   * pose and planning group*/
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup *joint_model_group =
      robot_state->getJointModelGroup(PLANNING_GROUP);

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of the world
  // (including the robot).
  planning_scene::PlanningScenePtr planning_scene(
      new planning_scene::PlanningScene(robot_model));

  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(
      joint_model_group, "ready");

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
      planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try {
    planner_plugin_loader.reset(
        new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
  } catch (pluginlib::PluginlibException &ex) {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader "
                     << ex.what());
  }
  try {
    planner_instance.reset(
        planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '"
                    << planner_instance->getDescription() << "'");
  } catch (pluginlib::PluginlibException &ex) {
    const std::vector<std::string> &classes =
        planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto &cls : classes)
      ss << cls << " ";
    ROS_ERROR_STREAM("Exception while loading planner '"
                     << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }


    ros::spin()
    return 0;
}