#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from husky_train.srv import EePose, EePoseResponse, EeRpy, EeRpyResponse, EeTraj, EeTrajResponse, JointTraj, JointTrajResponse

class ExecTrajService(object):

    def __init__(self):

        print("******************")
        print("moveit exectute service node")	

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.02)
        self.group.set_planning_time(5.0)
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_num_planning_attempts(5)

        self.ee_traj_srv = rospy.Service('/ee_traj_srv', EeTraj , self.ee_traj_callback)
        self.joint_traj_srv = rospy.Service('/joint_traj_srv', JointTraj , self.joint_traj_callback)
        self.ee_pose_srv = rospy.Service('/ee_pose_srv', EePose , self.ee_pose_callback)
        self.ee_rpy_srv = rospy.Service('/ee_rpy_srv', EeRpy , self.ee_rpy_callback)

        self.pose_target = geometry_msgs.msg.Pose()

    def ee_traj_callback(self, request):

        self.pose_target.orientation.w = request.pose.orientation.w
        self.pose_target.position.x = request.pose.position.x
        self.pose_target.position.y = request.pose.position.y
        self.pose_target.position.z = request.pose.position.z
        self.group.set_pose_target(self.pose_target)
        self.execute_trajectory()

        response = EeTrajResponse()
        response.success = True
        response.message = "Everything went OK"

        return response

    def joint_traj_callback(self, request):

        self.group_variable_values = self.group.get_current_joint_values()
        print ("Group Vars:")
        print (self.group_variable_values)
        print ("Point:")
        print (request.point.positions)
        self.group_variable_values[0] = request.point.positions[0]
        self.group_variable_values[1] = request.point.positions[1]
        self.group_variable_values[2] = request.point.positions[2]
        self.group_variable_values[3] = request.point.positions[3]
        self.group_variable_values[4] = request.point.positions[4]
        self.group_variable_values[5] = request.point.positions[5]
        # self.group_variable_values[6] = request.point.positions[6]
        self.group.set_joint_value_target(self.group_variable_values)
        self.execute_trajectory()

        response = JointTrajResponse()
        response.success = True
        response.message = "Everything went OK"

        return response

    def execute_trajectory(self):

        self.plan = self.group.plan()
        self.group.go(wait=True)

    def ee_pose_callback(self, request):

        gripper_pose = self.group.get_current_pose()

        gripper_pose_res = EePoseResponse()
        gripper_pose_res = gripper_pose.pose

        return gripper_pose_res

    def ee_rpy_callback(self, request):

        gripper_rpy = self.group.get_current_rpy()
        gripper_rpy_res = EeRpyResponse()
        gripper_rpy_res.r = gripper_rpy[0]
        gripper_rpy_res.y = gripper_rpy[1]
        gripper_rpy_res.p = gripper_rpy[2]

        return gripper_rpy_res

    def find_best_plan(self, pose):
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(pose)
        best_plan = temp_plan = self.group.plan()
        temp_num = best_num = 0
        loops = 100;
        for i in range(100):
            success = self.group.plan(temp_plan)
            if success:
                




if __name__ == "__main__":

    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    traj_serv_object = ExecTrajService()
    rospy.spin() # mantain the service open.
