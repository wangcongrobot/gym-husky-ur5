#include "motion_planner/motion_planner.h"

JRCMotionPlanner::JRCMotionPlanner(ros::NodeHandle &nh)
    : nh_(nh), 
    group_name_("right_arm"), 
    planning_id_("RRTConnectkConfigDefault"), 
    planning_attempts_(1),
      position_tolerance_(0.01),                                                       // 1cm
      orientation_tolerance_(0.02),                                                    // 0.57 * 2 deg
      planning_time_(0.1),                                                             // 0.1s
      max_vel_scale_factor_(0.5),                                                      // move_group
      jump_threshold_(2.0), 
      trajectory_velocity_scaling_(TRAJECTORY_VELOCITY_SCALING), // trajectory processing
      max_plan_steps_(MAX_PLAN_STEP), 
      max_cartesion_plan_steps_(MAX_CART_PLAN_STEP),
      // Get current joint state from the topic
    //   kinova_driver_joint_state_topic_("j2n6s300_driver/out/joint_state"),
      // Get current end effector pose from the topic
    //   kinova_driver_tool_pose_topic_("j2n6s300_driver/out/tool_pose"), 
      joint_states_topic_("joint_states")
{
	nh_.getParam("debug_print", debug_print_);
	nh_.getParam("confirm_act", confirm_act_);
	init();

    ee_traj_srv_ = nh_.advertiseService("/ee_traj_srv", &JRCMotionPlanner::EeTrajCallback, this);
    joint_traj_srv_ = nh_.advertiseService("/joint_traj_srv", &JRCMotionPlanner::JointTrajCallback, this);
    ee_pose_srv_ = nh_.advertiseService("/ee_pose_srv", &JRCMotionPlanner::EePoseCallback, this);
    ee_rpy_srv_ = nh_.advertiseService("/ee_rpy_srv", &JRCMotionPlanner::EeRpyCallback, this);
}

void JRCMotionPlanner::init()
{
	// MoveIt
	group_ = new moveit::planning_interface::MoveGroupInterface(group_name_);
	// MoveIt configuration
	group_->setGoalPositionTolerance(position_tolerance_);
	group_->setGoalOrientationTolerance(orientation_tolerance_);
	group_->setPlannerId(planning_id_);
	group_->setPlanningTime(planning_time_);
	group_->setMaxVelocityScalingFactor(max_vel_scale_factor_);
	group_->setNumPlanningAttempts(planning_attempts_);
}

JRCMotionPlanner::~JRCMotionPlanner() { delete group_; }

std::vector<double> JRCMotionPlanner::getCurrentJointState()
{
    if (debug_print_) {
        std::cout << "Enter JRCMotionPlanner::getCurrentJointValues" << std::endl;
    }

    sensor_msgs::JointStateConstPtr joint_state
        = ros::topic::waitForMessage<sensor_msgs::JointState>(
            joint_states_topic_, nh_, ros::Duration(1.0));
    if (!joint_state) throw std::runtime_error("Joint state message capture failed");

    if (debug_print_) {
        std::size_t num = joint_state->position.size();
        std::vector<double> joint;
        for (int i = 0; i < num; i++) {
            joint.push_back(joint_state->position[i]);
            std::cout << joint[i] << std::endl;
        }
        std::cout << "Leave JRCMotionPlanner::getCurrentJointValues" << std::endl;
    }
    std::vector<double> joint_values;
    for(int i=0;i<6;i++)
    {
        joint_values.push_back(joint_state->position[i]);
    }
    return joint_values;
}

bool JRCMotionPlanner::EePoseCallback(husky_train::EePose::Request& req,
                                      husky_train::EePose::Response& res)
{
    res.pose = group_->getCurrentPose().pose;
    return true;
}
bool JRCMotionPlanner::EeRpyCallback(husky_train::EeRpy::Request& req, 
                                     husky_train::EeRpy::Response& res)
{
    std::vector<double> rpy = group_->getCurrentRPY();
    res.r = rpy[0];
    res.p = rpy[1];
    res.y = rpy[2];
    return true;
}

bool JRCMotionPlanner::EeTrajCallback(husky_train::EeTraj::Request& req,
                                      husky_train::EeTraj::Response& res)
{
    group_->setStartStateToCurrentState();
    group_->setPoseTarget(req.pose);

    moveit::planning_interface::MoveGroupInterface::Plan temp_plan, best_plan;
    int loops = 100;
    ros::Duration best_time(100.0);
    ros::Duration current_time(0.0);
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(5.0);
    for (int i = 0; i < loops; i++)
    {
        moveit::planning_interface::MoveItErrorCode suc = group_->plan(temp_plan);
        if (suc)
        {
            current_time = temp_plan.trajectory_.joint_trajectory.points.back().time_from_start;
            if (current_time < best_time)
            {
                best_plan = temp_plan;
                best_time = current_time;
                if (debug_print_)
                {
                    std::cout << current_time << std::endl;
                }
            }
        }
        if ((ros::Time::now() - start_time) > timeout){
            break;
            ROS_ERROR("No solution in 5s!");
        }
    }
    std::cout << "Motion planning 100 times duration: " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

    bool plan_valid = false;
    int plan_steps = 0;

    plan_steps = getPlanPointNum(best_plan);
    if (plan_steps < max_plan_steps_){
        std::cout << "Plan found in " << best_plan.planning_time_ << " seconds with " << plan_steps << " steps" << std::endl;
        plan_valid = true;
    }
    if (!plan_valid){
        ROS_ERROR_STREAM("Plan found in " << plan_steps << " steps");
        exit(0);
    }
    // Execute the plan
    confirmToAct(req.pose);
    executePlan(best_plan);

    res.success = true;
    res.message = "Everything went OK";
}

std::size_t JRCMotionPlanner::getPlanPointNum(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
    return trajectory.joint_trajectory.points.size();
}

void JRCMotionPlanner::setJointValueTarget(const std::vector<double> joint_values)
{
    group_->setJointValueTarget(joint_values);
    confirmToAct();
    group_->move();
}

bool JRCMotionPlanner::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    ros::Time start_time = ros::Time::now();
    group_->execute(plan);
    std::cout << "Motion execute duration: " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
    std::cout << "\n MOVE TO TARGET SUCCESSFULLY\n" << std::endl;
}


bool JRCMotionPlanner::JointTrajCallback(husky_train::JointTraj::Request& req,
                                         husky_train::JointTraj::Response& res)
{
    // std::vector<double> joint_target;
    // for (int i = 0; i < 6; i++){
    //     joint_target[i] = req.point.position[i]
    // }
    // setJointValueTarget(joint_target);
    
    res.success = true;
    res.message = "Everything went OK";

    return true;
}

void JRCMotionPlanner::confirmToAct()
{
    std::cout << "Confirm start and end info and press n to start plan" << std::endl;
    if (confirm_act_) 
    {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_ || "N" == pause_) {
            std::cout << "Valid info, begin to move" << std::endl;
        } else {
            return;
        }
    }
}

void JRCMotionPlanner::confirmToAct(const geometry_msgs::Pose& goal)
{
    ROS_INFO_STREAM("Move to target" << goal);
    std::cout << "Confirm start ---> goal info, press n to start plan" << std::endl;
    if (confirm_act_) {
        std::string pause_;
        std::cin >> pause_;
        if ("n" == pause_) {
            std::cout << "Correct state, begin to plan" << std::endl;
        } else {
            return;
        }
    }
}