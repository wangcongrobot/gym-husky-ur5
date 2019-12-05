#include "motion_planner/kinematics_parser.h"


#include "motion_planner/motion_planner.h"


int evaluateMoveitPlan(moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    moveit_msgs::RobotTrajectory trajectory = plan.trajectory_;
    // std::vector<trajectory_msgs::JointTrajectoryPoint> points =
    // trajectory.joint_trajectory.points;
    return trajectory.joint_trajectory.points.size();
}
// Cartesian line plan
void moveLineTarget(const geometry_msgs::Pose& goal)
{
    ROS_INFO("Begin cartesian line plan");
    moveit::planning_interface::MoveGroupInterface group("left_arm");
    group.setGoalPositionTolerance(0.01);    // 3cm
    group.setGoalOrientationTolerance(0.02); // 5.729576129 * 2 deg
    group.setMaxVelocityScalingFactor(0.5);
    group.allowReplanning(true);
    group.setPlanningTime(5.0);

    geometry_msgs::PoseStamped current = group.getCurrentPose();
    geometry_msgs::Pose start = current.pose;
    geometry_msgs::Pose way_pose = start;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(way_pose); // first pose waypoint
    int num_waypoint = 6;
    float delta_x = (goal.position.x - start.position.x) / (num_waypoint - 1);
    float delta_y = (goal.position.y - start.position.y) / (num_waypoint - 1);
    float delta_z = (goal.position.z - start.position.z) / (num_waypoint - 1);

    // interplotate between current pose and target pose
    for (int i = 0; i < num_waypoint - 1; i++) {
        way_pose.position.x += delta_x;
        way_pose.position.y += delta_y;
        way_pose.position.z += delta_z;
        waypoints.push_back(way_pose);
        ROS_INFO_STREAM(way_pose);
    }


    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.02;
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction == 1.0) {
        std::cout << "computeCartesionPath Successfully" << std::endl;
    } else if (fraction == -1.0) {
        ROS_ERROR("computeCartesionPath ERROR!");
    } else {
        ROS_ERROR_STREAM("computeCartesionPath : " << fraction * 100 << " %");
    }
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;

    int plan_steps = evaluateMoveitPlan(cartesian_plan);
    ROS_INFO_STREAM("Line plan steps: " << plan_steps);
    if (plan_steps < 100) {
        ROS_INFO_STREAM("Plan found in " << cartesian_plan.planning_time_ << " seconds with "
                                         << plan_steps << " steps");
        group.execute(cartesian_plan);
    } else {
        exit(0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_parser_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    double x, y, z, num1, num2;
    nh.getParam("x", x);
    nh.getParam("y", y);
    nh.getParam("z", z);
    nh.getParam("num1", num1);
    nh.getParam("num2", num2);
    std::cout << x << y << z << num1 << num2 << std::endl;

    Parser parser_;
    JRCMotionPlanner motion_planner(nh);

    bool debug_print_ = true;
    double trajectory_velocity_scaling_ = 0.5;
    moveit::planning_interface::MoveGroupInterface group_("left_arm");
    std::vector<std::string> left_arm_joint_names_ = {"l_ur5_arm_elbow_joint", "l_ur5_arm_shoulder_lift_joint", "l_ur5_arm_shoulder_pan_joint", "l_ur5_arm_wrist_1_joint", "l_ur5_arm_wrist_2_joint", "l_ur5_arm_wrist_3_joint"};
    std::vector<std::string> right_arm_joint_names_ = {"r_ur5_arm_elbow_joint", "r_ur5_arm_shoulder_lift_joint", "r_ur5_arm_shoulder_pan_joint", "r_ur5_arm_wrist_1_joint", "r_ur5_arm_wrist_2_joint", "r_ur5_arm_wrist_3_joint"};
    std::string use_arm_ = "left";
    std::string moveit_traj_arm_base_frame_ = "l_ur5_base_link";


    std::vector<double> current_joint_values = motion_planner.getCurrentJointStateFromMoveit();
    // ROS_INFO_STREAM(current_joint_values);
    for (std::vector<double>::const_iterator i = current_joint_values.begin(); i != current_joint_values.end(); ++i){
        std::cout << *i << ' ' ;
    }
    std::cout << std::endl;
    for (int i =0; i < current_joint_values.size(); i++){
        std::cout << current_joint_values.at(i) << ' ';
    }
    std::cout << std::endl;

    std::vector<double> joint_values = {0.861787405574,
                                        -1.54073192249, 
                                        1.4611893835, 
                                        0.058000607052, 
                                        0.841070054649, 
                                        -0.874129776051};
    motion_planner.setJointValueTarget(joint_values);


    double distance_x = x; 
    double distance_y = y; 
    double distance_z = z;
    int number_point = num1;
    int number_distance = num2;

	std::cout << "cartesion path planner..." << std::endl;

	Eigen::VectorXd qPre(6); // NOT Eigen::VectorXd qPre!!! Must be qPre(6)

	// std::vector<double> joint_recv = getCurrentJointState();
    std::vector<double> joint_recv = motion_planner.getCurrentJointStateFromMoveit();

	qPre << joint_recv[0], joint_recv[1], joint_recv[2], joint_recv[3], joint_recv[4], joint_recv[5];
	if (debug_print_)
	{
		std::cout << "qPre: "
		          << "\n"
		          << qPre << std::endl;
	}

	// Current transformation, including T & R
	Eigen::Matrix4d transformation = parser_.Foward(qPre);
	// if(debug_print_)
	{
		std::cout << "current joint values: " << "\n" << qPre << std::endl;
		std::cout << "current transformation1: "
		          << "\n"
		          << transformation << std::endl;
	}

	// FollowJointTrajectoryActionGoal
	control_msgs::FollowJointTrajectoryGoal follow_joint_traj_goal;
	moveit_msgs::RobotTrajectory            moveit_robot_traj_msg;

	follow_joint_traj_goal.trajectory.header.frame_id = moveit_traj_arm_base_frame_;
	follow_joint_traj_goal.trajectory.header.stamp    = ros::Time::now();
	follow_joint_traj_goal.trajectory.joint_names.clear();
	moveit_robot_traj_msg.joint_trajectory.header.frame_id = moveit_traj_arm_base_frame_;
	moveit_robot_traj_msg.joint_trajectory.header.stamp    = ros::Time::now();
	moveit_robot_traj_msg.joint_trajectory.joint_names.clear();

	for (int k = 0; k < 6; k++) {
		std::stringstream jointName;
		// jointName << "j2n6s300_joint_" << (k + 1);
		// follow_joint_traj_goal.trajectory.joint_names.push_back(jointName.str());
		// moveit_robot_traj_msg.joint_trajectory.joint_names.push_back(jointName.str());
        if (use_arm_ == "left"){
            jointName << left_arm_joint_names_[k];
		    follow_joint_traj_goal.trajectory.joint_names.push_back(jointName.str());
		    moveit_robot_traj_msg.joint_trajectory.joint_names.push_back(jointName.str());
        }
        if (use_arm_ == "right"){
            jointName << right_arm_joint_names_[k];
		    follow_joint_traj_goal.trajectory.joint_names.push_back(jointName.str());
		    moveit_robot_traj_msg.joint_trajectory.joint_names.push_back(jointName.str());
        }
	}

	follow_joint_traj_goal.trajectory.points.clear();
	moveit_robot_traj_msg.joint_trajectory.points.clear();

	ros::Time     start_time = ros::Time::now();
	ros::Duration timeout(1.0);
	for (std::size_t i = 0; i < number_point; i++) {
		// translation
		transformation(0, 3) += distance_x / number_point;
		transformation(1, 3) += distance_y / number_point;
		transformation(2, 3) += distance_z / number_point;

		// IK from transformation()
		Eigen::VectorXd q(6);
		q = parser_.Inverse(transformation, qPre);
		//        std::cout << "=========" << std::endl;
		if ((ros::Time::now() - start_time) > timeout)
		{
			ROS_ERROR("Planning Timeout!");
			break;
		}
		if (q(0) > 10)
			continue;

		// printf("loops:%d",(int)i);
		for (std::size_t j = 0; j < number_distance; j++) {
			trajectory_msgs::JointTrajectoryPoint point;
			for (int k = 0; k < 6; k++) {
				point.positions.push_back(qPre(k) + (q(k) - qPre(k)) / number_distance * j);
				point.velocities.push_back(0.0);
				point.accelerations.push_back(0.0);
			}
			point.time_from_start = ros::Duration();
			follow_joint_traj_goal.trajectory.points.push_back(point);
			moveit_robot_traj_msg.joint_trajectory.points.push_back(point);
		}
		// printf("\n\njoint values : %d\n",(int)i);
        if (debug_print_){
            // std::cout << q  << "\n" << std::endl;
        }
		
		qPre = q;
		if ((ros::Time::now() - start_time) > timeout)
		{
			ROS_ERROR("Planning Timeout!");
			break;
		}
		if (debug_print_)
		{
			std::cout << "Planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;
		}
	}
	if(debug_print_)
	{
		std::cout << "target joint values: "  << "\n" << qPre << std::endl;
		std::cout << "target transformation: "
		          << "\n"
		          << transformation << std::endl;
	}
	std::cout << "Total planning time is : " << (ros::Time::now() - start_time).toSec() << "s" << std::endl;

	double result =
	    (double)moveit_robot_traj_msg.joint_trajectory.points.size() / (double)(number_point * number_distance);
    if (debug_print_){
        std::cout << "trajectory points number: " << moveit_robot_traj_msg.joint_trajectory.points.size() << ", "
	          << result * 100 << "%" << std::endl;
    }
	
	if (result == 0.0)
	{
		ROS_ERROR("compute cartesion path is ERROR!");
	}
	else if (result == 1.0)
	{
		std::cout << "compute cartesion path successfully! : 100 %" << std::endl;
	}
	else if (result > 0.0 && result < 1.0)
	{
		ROS_ERROR_STREAM("compute cartesion path : " << result * 100 << "%");
	}
	//    addTimeToTraj(&moveit_robot_traj_msg, TRAJECTORY_VELOCITY_SCALING);
	robot_trajectory::RobotTrajectory rt(group_.getCurrentState()->getRobotModel(), group_.getName());
	rt.setRobotTrajectoryMsg(*group_.getCurrentState(), moveit_robot_traj_msg);
	trajectory_processing::IterativeParabolicTimeParameterization iptp;

	bool IptpSuccess = false;
	IptpSuccess      = iptp.computeTimeStamps(rt, trajectory_velocity_scaling_);
	if (!IptpSuccess)
	{
		ROS_ERROR("Computed time stamped FAILED");
	}
	rt.getRobotTrajectoryMsg(moveit_robot_traj_msg);
    if (debug_print_){
        // ROS_INFO_STREAM(moveit_robot_traj_msg);
    }

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = moveit_robot_traj_msg;
	motion_planner.confirmToAct();
    // ROS_INFO_STREAM(plan);
	// motion_planner.executePlan(plan);
	motion_planner.executeTrajectory(moveit_robot_traj_msg.joint_trajectory);


    ros::shutdown();
    return 0;
}
