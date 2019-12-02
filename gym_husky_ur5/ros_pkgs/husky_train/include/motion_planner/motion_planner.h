#ifndef JRC18SIA_MOTION_PLANNER_H
#define JRC18SIA_MOTION_PLANNER_H

// C++ STL
#include <vector>
#include <string>
#include <iostream>

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

// tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

// ROS message
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>

// custom
// #include "jrc18sia_motion_planner/jrc18sia_kinematics_parser.h"

// custom ROS Service
#include <husky_train/EePose.h>
#include <husky_train/EePoseResponse.h>
#include <husky_train/EeRpy.h>
#include <husky_train/EeRpyResponse.h>
#include <husky_train/EeTraj.h>
#include <husky_train/EeTrajResponse.h>
#include <husky_train/JointTraj.h>
#include <husky_train/JointTrajResponse.h>
// from husky_train.srv import EePose, EePoseResponse, EeRpy, EeRpyResponse, EeTraj, EeTrajResponse, JointTraj, JointTrajResponse

#define DEBUG_PRINT true
#define CONFIRM_ACT true
#define TRAJECTORY_VELOCITY_SCALING 1.0 // trajectory_velocity_scaling_;
#define MAX_PLAN_STEP 30 // max_plan_steps_
#define MAX_CART_PLAN_STEP 200 //max_cartesion_plan_steps_

class JRCMotionPlanner
{
public:

  /** \brief Constructor */
  JRCMotionPlanner(ros::NodeHandle &nh);

  /** \brief Destructor */
  ~JRCMotionPlanner();

  /**
   * \brief Calculate the cartesion trajectory
   * \param distance_x, distance_y, distance_z : relative distance from current pose in cartesion space
   * \param roll, pitch, yaw absolute RPY : angle for fixed axis X, Y, Z
   * \param number_point, number_distance : number_point in cartesion space, number_distance in joint space
   * \return successful percentage
    */
  double cartesionPathPlanner(double distance_x, double distance_y, double distance_z,
                                double roll, double pitch, double yaw,
                                int number_point, int number_distance);

  /**
   * \brief Calculate the cartesion trajectory
   * \param distance_x, distance_y, distance_z : relative distance from current pose in cartesion space
   * \param roll, pitch, yaw absolute RPY : angle for fixed axis X, Y, Z
   * \return successful percentage
    */
  double cartesionPathPlanner(double distance_x, double distance_y, double distance_z,
                                double roll, double pitch, double yaw);
  /**
   * \brief Calculate the cartesion trajectory, move line
   * \param distance_x, distance_y, distance_z : relative distance from current pose in cartesion space
   * \param number_point, number_distance : number_point in cartesion space, number_distance in joint space
   * \return successful percentage
    */
  double cartesionPathPlanner(double distance_x, double distance_y, double distance_z,
                                int number_point, int number_distance);

  /**   * \brief Calculate the cartesion trajectory, move line
   * \param distance_x, distance_y, distance_z : relative distance from current pose in cartesion space
   * \return successful percentage
    */
  double cartesionPathPlanner(double distance_x, double distance_y, double distance_z);

  /** \brief Move to target */
  void moveToTargetBestTime(const geometry_msgs::Pose& target);

  /** \brief Move to target */
  void moveToTargetBestTime(const geometry_msgs::PoseStamped& target);

  /** \brief Move to target */
  void moveToTargetNamed(const std::string& target_name);

  /** \brief Move line in cartesion space by using Moveit cartesion planner */
  void moveLineTarget(const geometry_msgs::Pose& goal);

  /** \brief Move line in cartesion space by using Moveit cartesion planner */
  void moveLineTarget(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);

  /** \brief Set absolute joint values to move */
  void setJointValueTarget(const std::vector<double> joint_values);

  /** \brief  Set joint index and relative joint values to move */
  void setJointValueTarget(const int joint_index, const double joint_values);

  void setAbsoluteJointValueTarget(const int joint_index, const double joint_values);

  /** \brief  Get current joint values from the topic "joint_states" */
  std::vector<double> getCurrentJointState();

  /** \brief Get end-effector pose from kinova driver topic j2n6s300_driver/out/tool_pose */
//  geometry_msgs::PoseStamped getCurrentPose();

  /** \brief  Get current joint values from the topic "joint_states" */
  std::vector<double> getCurrentJointStateFromMoveit();

  geometry_msgs::Pose getCurrentPoseFromMoveit();

  /** \brief Get end-effector pose from kinova driver topic j2n6s300_driver/out/tool_pose */
  geometry_msgs::Pose getCurrentPoseFromDriver();

  /** \brief Get the roll-pitch-yaw (XYZ) for the end-effector \e end_effector_link.
      If \e end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is
     assumed */
  std::vector<double> getCurrentRPY();

  /** \brief Enter 'n' to confirm to move */
  void confirmToAct(
      const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal, const std::string& str);

  /** \brief Enter 'n' to confirm to move */
  void confirmToAct(const geometry_msgs::Pose& goal, const std::string& str);

  /** \brief Enter 'n' to confirm to move */
  void confirmToAct();

// private:
  /** \brief Initialize the variables */
  void init();

  /** \brief Execute the trajectory in MoveIt! and print the duration time*/
  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

  bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
  {
    // Create a Follow Joint Trajectory Action Client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client("/j2n6s300/follow_joint_trajectory", true);
    if(!action_client.waitForServer(ros::Duration(2.0)))
    {
      ROS_ERROR("Cannot connect to action server");
      return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    goal.goal_time_tolerance = ros::Duration(1.0);

    return action_client.sendGoalAndWait(goal) == actionlib::SimpleClientGoalState::SUCCEEDED;
  }

  /**
   * \brief Evaluate the moveit plan trajecotry
   * \param plan : moveit plan
   * \return the number of the trajectory point
    */
  std::size_t getPlanPointNum(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

  /** \brief Enter 'n' to confirm to move */
  void confirmToAct(
      const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal);

  /** \brief Enter 'n' to confirm to move */
  void confirmToAct(const geometry_msgs::Pose& goal);

private:
  ros::NodeHandle nh_;
  ros::Duration timeout_;

  // Custom FK & IK of Kinova Jaco2 arm
//   Parser parser_;

  // MoveIt
  moveit::planning_interface::MoveGroupInterface *group_;
  std::string kinova_driver_joint_state_topic_; // get current joint state from the topic
  std::string kinova_driver_tool_pose_topic_; // get current end effector pose from the topic
  std::string joint_states_topic_;
  std::string moveit_pose_topic_;
  std::string group_name_;
  std::vector<std::string> joint_names_;

  std::string class_file_name_;
  std::string address;
  std::string robot_type_;

  /***Get parameters from jrc18sia_motion_planner_parameters.yaml***/
  // MoveIt config
  double position_tolerance_;
  double orientation_tolerance_;
  double planning_time_;
  double max_vel_scale_factor_;
  double planning_attempts_;
  std::string planning_id_;

  // MoveIt cartesionPath
  double jump_threshold_;

  // trajectory processing config
  double trajectory_velocity_scaling_;

  // Evaluate the plan trajectory
  double max_plan_steps_;
  double max_cartesion_plan_steps_;

  // Debug setting
  bool debug_print_;
  bool confirm_act_;

    ros::ServiceServer ee_traj_srv_;
    ros::ServiceServer joint_traj_srv_;
    ros::ServiceServer ee_pose_srv_;
    ros::ServiceServer ee_rpy_srv_;
    geometry_msgs::Pose pose_target_;

    bool EeTrajCallback(husky_train::EeTraj::Request& req,
                        husky_train::EeTraj::Response& res);
    bool JointTrajCallback(husky_train::JointTraj::Request& req,
                           husky_train::JointTraj::Response& res);
    bool EePoseCallback(husky_train::EePose::Request& req,
                        husky_train::EePose::Response& res);
    bool EeRpyCallback(husky_train::EeRpy::Request& req,
                       husky_train::EeRpy::Response& res);

};

#endif // MOTION_PLANNER_H

