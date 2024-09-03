#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function

import rospy
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import time
import math

# Global variables
right_trajectory = DisplayTrajectory()
right_open = 0.0

left_open = 0.0
left_trajectory = DisplayTrajectory()

right_trajectory_receive = 0
left_trajectory_receive = 0

def gripper_right_callback(msg):
    global right_open
    rospy.loginfo("Received right gripper value: %f", msg.data)
    right_open = msg.data

def gripper_left_callback(msg):
    global left_open
    rospy.loginfo("Received left gripper value: %f", msg.data)
    left_open = msg.data

def right_display_trajectory_callback(msg):
    global right_trajectory, right_trajectory_receive
    rospy.loginfo("Number of trajectories: %d", len(msg.trajectory))

    if len(msg.trajectory) > 0:
        right_trajectory_receive += 1

    for i, traj in enumerate(msg.trajectory):
        rospy.loginfo("[RIGHT] Trajectory %d", i)
        rospy.loginfo("  Number of points: %d", len(traj.joint_trajectory.points))

    right_trajectory = msg

    if right_trajectory.trajectory and right_trajectory.trajectory[0].joint_trajectory.points:
        right_trajectory.trajectory[0].joint_trajectory.points.pop(0)

def left_display_trajectory_callback(msg):
    global left_trajectory, left_trajectory_receive
    rospy.loginfo("Number of trajectories: %d", len(msg.trajectory))

    if len(msg.trajectory) > 0:
        left_trajectory_receive += 1

    for i, traj in enumerate(msg.trajectory):
        rospy.loginfo("[LEFT] Trajectory %d", i)
        rospy.loginfo("  Number of points: %d", len(traj.joint_trajectory.points))

    left_trajectory = msg

    if left_trajectory.trajectory and left_trajectory.trajectory[0].joint_trajectory.points:
        left_trajectory.trajectory[0].joint_trajectory.points.pop(0)


def main():
    rospy.init_node('motion_planning')

    joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.Subscriber('/move_group/left_display_planned_path', DisplayTrajectory, left_display_trajectory_callback)
    rospy.Subscriber('/move_group/right_display_planned_path', DisplayTrajectory, right_display_trajectory_callback)
    rospy.Subscriber('gripper_left', Float32, gripper_left_callback)
    rospy.Subscriber('gripper_right', Float32, gripper_right_callback)

    loop_rate = rospy.Rate(200)

    last_right_open = right_open
    last_left_open = left_open
    left_traject_receive = left_trajectory_receive
    right_traject_receive = right_trajectory_receive

    joint_names = [
        "left_1_joint", "left_2_joint", "left_3_joint", "left_4_joint",
        "left_5_joint", "left_6_joint", "left_7_joint", "left_8_joint",
        "right_1_joint", "right_2_joint", "right_3_joint", "right_4_joint",
        "right_5_joint", "right_6_joint", "right_7_joint", "right_8_joint"
    ]

    joint_pos_minus = [
        -1, -1, -1, -1,
        1, 1, 1, -1,
        -1, 1, -1, -1,
        1, 1, 1, -1,
    ]

    joint_ids = [
        7, 6, 5, 4,
        3, 2, 1, 0,
        8, 9, 10, 11,
        12, 13, 14, 15
    ]

    rospy.spin()

if __name__ == '__main__':
    main()
