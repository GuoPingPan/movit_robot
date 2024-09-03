#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

def rpy_to_quaternion(roll, pitch, yaw):
    # 将输入的角度转换为弧度
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    
    # 计算四元数的各部分
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    return [qx, qy, qz, qw]


import math

def quaternion_to_rpy(q):
    x, y, z, w = q

    # 计算Roll (绕X轴旋转)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # 计算Pitch (绕Y轴旋转)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # 用90度代替不稳定的值
    else:
        pitch = math.asin(sinp)

    # 计算Yaw (绕Z轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # 将弧度转换为角度
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    # # 确保结果在 -180° 到 180° 范围内
    # roll = (roll + 180) % 360 - 180
    # pitch = (pitch + 180) % 360 - 180
    # yaw = (yaw + 180) % 360 - 180

    return roll, pitch, yaw

fake = True

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("bimanipulation", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()
        self.scene = scene

        group_name = "left_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group_left = move_group
        group_name = "right_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group_right = move_group
        

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.left_display_trajectory_publisher = rospy.Publisher(
            "/move_group/left_display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )
        self.right_display_trajectory_publisher = rospy.Publisher(
            "/move_group/right_display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        self.left_gripper_pub = rospy.Publisher("gripper_left", Float32, queue_size=1)
        self.right_gripper_pub = rospy.Publisher("gripper_right", Float32, queue_size=1)

        self.joint_control_pos_minus = [
            1, -1, 1, 1,
            -1, -1, 1, 1,
            -1, -1, -1, -1,
            1, 1, -1, -1,
        ]

        self.left_ee_origin = [0, 0.1878, -0.375]
        self.right_ee_origin = [0, -0.1878, -0.375]
        
        # We can get the name of the reference frame for this robot:
        print("============ Planning frame: %s" % self.move_group_left.get_planning_frame())
        print("============ End effector link: %s" % self.move_group_left.get_end_effector_link())
        print("============ Planning frame: %s" % self.move_group_right.get_planning_frame())
        print("============ End effector link: %s" % self.move_group_right.get_end_effector_link())
        print("============ Available Planning Groups:", self.robot.get_group_names())
        print("============ Printing robot state")
        
        # ============ Planning frame: base_link
        # ============ End effector link: left_7_link
        # ============ Planning frame: base_link
        # ============ End effector link: right_7_link
        # ============ Available Planning Groups: ['left_arm', 'left_arm', 'right_arm', 'right_arm']
        # ============ Printing robot state
        
        # print(self.robot.get_current_state())
        print("")
        # import ipdb; ipdb.set_trace()



    def plan_to_pose_goal(self, hand_name, xyzrpy):

        input("start plan")

        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_7_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
            origin_xyz = self.left_ee_origin

        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_7_link"
            display_trajectory_publisher = self.right_display_trajectory_publisher
            origin_xyz = self.right_ee_origin
        else:
            raise NotImplementedError("the hand name not in move_group")
        
        # https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html
        move_group.set_planning_pipeline_id('ompl')
        move_group.set_planner_id('manipulator[RRTConnect]')
        # move_group.set_goal_position_tolerance(0.001)
        # move_group.set_goal_orientation_tolerance(0.01)
        # move_group.set_planning_time(1.0)
        # move_group.set_num_planning_attempts(10)
        # move_group.set_max_velocity_scaling_factor(0.1)
        # move_group.set_max_acceleration_scaling_factor(0.1)

        move_group.clear_pose_targets()
     
        pose_goal = move_group.get_current_pose(end_effector_link).pose
        pose_goals = []

        for (x, y, z, r, p, y) in xyzrpy:
            x += origin_xyz[0]
            y += origin_xyz[1]
            z += origin_xyz[2]
            qx, qy, qz, qw = rpy_to_quaternion(r, p, y)

            pose_goal_temp = copy.deepcopy(pose_goal)
            pose_goal_temp.position.x = x
            pose_goal_temp.position.y = y
            pose_goal_temp.position.z = z

            pose_goal_temp.orientation.x = qx
            pose_goal_temp.orientation.y = qy
            pose_goal_temp.orientation.z = qz
            pose_goal_temp.orientation.w = qw

            pose_goals.append(pose_goal_temp)
            
        print(pose_goals)
        move_group.clear_pose_targets()
        move_group.set_pose_targets(pose_goals, end_effector_link)
        success, plan, fraction, others = move_group.plan()

        if not success:
            print("Fail to arive.")
            return

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        if fake:
            move_group.execute(plan, wait=True)

        # move_group.execute(plan, wait=True)


    def plan_to_joint(self, hand_name, joint_values):

        input("start plan")

        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_7_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_7_link"
            display_trajectory_publisher = self.right_display_trajectory_publisher
        else:
            raise NotImplementedError("the hand name not in move_group")

        move_group.clear_pose_targets()
        joint_goal = move_group.get_current_joint_values()

        if hand_name == "left_arm":
            joint_goal[0] = joint_values[0] * self.joint_control_pos_minus[0]
            joint_goal[1] = joint_values[1] * self.joint_control_pos_minus[1]
            joint_goal[2] = joint_values[2] * self.joint_control_pos_minus[2]
            joint_goal[3] = joint_values[3] * self.joint_control_pos_minus[3]
            joint_goal[4] = joint_values[4] * self.joint_control_pos_minus[4]
            joint_goal[5] = joint_values[5] * self.joint_control_pos_minus[5]
            joint_goal[6] = joint_values[6] * self.joint_control_pos_minus[6]
            # joint_goal[7] = joint_values[7] * self.joint_control_pos_minus[7]
        elif hand_name == "right_arm":
            joint_goal[0] = joint_values[0] * self.joint_control_pos_minus[8]
            joint_goal[1] = joint_values[1] * self.joint_control_pos_minus[9]
            joint_goal[2] = joint_values[2] * self.joint_control_pos_minus[10]
            joint_goal[3] = joint_values[3] * self.joint_control_pos_minus[11]
            joint_goal[4] = joint_values[4] * self.joint_control_pos_minus[12]
            joint_goal[5] = joint_values[5] * self.joint_control_pos_minus[13]
            joint_goal[6] = joint_values[6] * self.joint_control_pos_minus[14]
            # joint_goal[7] = joint_values[7] * self.joint_control_pos_minus[15]

        move_group.set_joint_value_target(joint_goal)
        success, plan, fraction, others = move_group.plan()

        if not success:
            print("Fail to arive.")
            return

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        if fake:
            move_group.execute(plan, wait=True)


    def plan_cartesian_path(self, hand_name, xyzrpy):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_7_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
            origin_xyz = self.left_ee_origin

        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_7_link"
            display_trajectory_publisher = self.right_display_trajectory_publisher
            origin_xyz = self.right_ee_origin
            
        else:
            raise NotImplementedError("the hand name not in move_group")
        
        
        move_group.clear_pose_targets()
        waypoints = []

        pose_goal = move_group.get_current_pose(end_effector_link).pose
        print(pose_goal)

        for (x, y, z, r, p, y) in xyzrpy:
            x += origin_xyz[0]
            y += origin_xyz[1]
            z += origin_xyz[2]
            qx, qy, qz, qw = rpy_to_quaternion(r, p, y)

            pose_goal_temp = copy.deepcopy(pose_goal)
            pose_goal_temp.position.x = x
            pose_goal_temp.position.y = y
            pose_goal_temp.position.z = z

            pose_goal_temp.orientation.x = qx
            pose_goal_temp.orientation.y = qy
            pose_goal_temp.orientation.z = qz
            pose_goal_temp.orientation.w = qw

            waypoints.append(pose_goal_temp)

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        print(self.robot.get_current_state())
        
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        if fake:
            move_group.execute(plan, wait=True)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def get_joint_states(self, hand_name):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            
        current_joints = move_group.get_current_joint_values()
        print(f"{hand_name} current_joints: ", current_joints)

        return current_joints

    def get_ee_pose(self, hand_name):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_7_link"
            origin_xyz = self.left_ee_origin
            
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_7_link"
            origin_xyz = self.right_ee_origin
            
        pose_goal = move_group.get_current_pose(end_effector_link).pose
        pose_goal_temp = copy.deepcopy(pose_goal)
        x = pose_goal_temp.position.x - origin_xyz[0]
        y = pose_goal_temp.position.y - origin_xyz[1]
        z = pose_goal_temp.position.z - origin_xyz[2]
        
        qw = pose_goal_temp.orientation.w
        qx = pose_goal_temp.orientation.x
        qy = pose_goal_temp.orientation.y
        qz = pose_goal_temp.orientation.z
        
        r, p, yaw = quaternion_to_rpy([qx, qy, qz, qw])
        
        print(f"============ [{hand_name}] ============")
        print(f" pose_goal: ", [x, y, z])
        print(f" rpy: ", [r, p, yaw])
        print("\n")
        
        return pose_goal_temp


    def control_gripper(self, hand_name, open):
        gripper_rad = 0
        if hand_name == 'left_arm':
            gripper_pub = self.left_gripper_pub
            if open:
                gripper_rad = -1 * self.joint_control_pos_minus[7]
        elif hand_name == 'right_arm':
            gripper_pub = self.right_gripper_pub
            if open:
                gripper_rad = -1 * self.joint_control_pos_minus[15]
        

        gripper_pub.publish(gripper_rad)            


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        tutorial = MoveGroupPythonInterfaceTutorial()

        # xyzrpy = [[0.1470199105271165, 0.017876022259465663, 0.12061123915518396, -0.01651869653088783, -45.38950309206009, 0.015158901667351699],
        #           [0.1470199105271165, 0.017876022259465663, 0.15061123915518396, -0.01651869653088783, -20.38950309206009, 0.015158901667351699],
        #           [0.1070199105271165, 0.117876022259465663, 0.15061123915518396, -0.01651869653088783, -45.38950309206009, 0.015158901667351699]]
      
        xyzrpy = [[0, 0, 0.05, 0, 0, 0]]
        tutorial.plan_to_pose_goal("left_arm", xyzrpy,)
        # # tutorial.plan_to_pose_goal("right_arm", xyzrpy)
        
        # joints = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, -0.2,]
        # tutorial.plan_to_joint("left_arm", joints)
        # # tutorial.plan_to_joint("right_arm", joints)
        
        # tutorial.plan_to_pose_goal("left_arm", xyzrpy)
        # tutorial.plan_to_joint("left_arm", joints)
   
        # xyzrpy = [[0.1470199105271165, 0.017876022259465663, 0.15061123915518396, -0.01651869653088783, -60.38950309206009, 0.015158901667351699]]
   
        # tutorial.plan_to_pose_goal("left_arm", xyzrpy)

        # xyzrpy = [[0.1470199105271165, 0.017876022259465663, 0.15061123915518396, -0.01651869653088783, -20.38950309206009, 0.015158901667351699]]
   
        # tutorial.plan_to_pose_goal("left_arm", xyzrpy)
        

        # xyzrpy = [[0.1470199105271165, -0.017876022259465663, 0.15061123915518396, -0.01651869653088783, -20.38950309206009, 0.015158901667351699]]

        # tutorial.plan_to_pose_goal("right_arm", xyzrpy)



        # xyzrpy = [[0.1470199105271165, -0.017876022259465663, 0.15061123915518396, -0.01651869653088783, -45.38950309206009, 0.015158901667351699]]

        # tutorial.plan_to_pose_goal("right_arm", xyzrpy)

        # tutorial.plan_cartesian_path("left_arm", xyzrpy)
        # tutorial.plan_cartesian_path("right_arm", xyzrpy)
        
        # tutorial.get_joint_states("left_arm")
        # tutorial.get_joint_states("right_arm")
        
        # for i in range(10):
        #     tutorial.get_ee_pose("left_arm")
        #     tutorial.get_ee_pose("right_arm")
        
        # while True:
        # for i in range(1000):
        #     tutorial.control_gripper("left_arm", open=1)
        #     tutorial.control_gripper("right_arm", open=1)

 
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
