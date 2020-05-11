#!/usr/bin/env python
import sys

import moveit_commander
import rospy
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander.conversions import pose_to_list
pi = 3.14159


def all_close(goal, actual, tolerance):

    if type(goal) is list:
        for g, a in zip(goal, actual):
            if abs(a-g) > tolerance:
                return False
    elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupNode:
    def __init__(self):
        rospy.init_node("tutorial")
        rospy.loginfo("Starting MoveGroupNode as tutorial.")

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_pub = rospy.Publisher(
            "move_group/display_planned_path", DisplayTrajectory, queue_size=20)

        planning_frame = move_group.get_planning_frame()
        rospy.loginfo("Planning frame: {}".format(planning_frame))

        eef_link = move_group.get_end_effector_link()
        rospy.loginfo("End effector link: {}".format(eef_link))

        group_names = robot.get_group_names()
        rospy.loginfo("Available Planning Groups: {}".format(group_names))

        rospy.loginfo("Robot state: {}".format(robot.get_current_state()))

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_pub = display_trajectory_pub
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0.0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0.0
        joint_goal[5] = pi/3
        joint_goal[6] = 0.0

        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


if __name__ == "__main__":
    tutorial = MoveGroupNode()
    tutorial.go_to_joint_state()
    rospy.spin()
