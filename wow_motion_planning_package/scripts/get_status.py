#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

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
        group_name = "left_gripper"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.gripper_left = move_group
        group_name = "right_gripper"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.gripper_right = move_group
        

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        left_display_trajectory_publisher = rospy.Publisher(
            "/move_group/left_display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )
        right_display_trajectory_publisher = rospy.Publisher(
            "/move_group/right_display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=1,
        )

        
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
        # ============ Available Planning Groups: ['left_arm', 'left_gripper', 'right_arm', 'right_gripper']
        # ============ Printing robot state
        
        print(self.robot.get_current_state())
        print("")
        import ipdb; ipdb.set_trace()

        self.left_display_trajectory_publisher = left_display_trajectory_publisher


    def plan_to_pose_goal(self, hand_name):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_8_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_8_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        else:
            raise NotImplementedError("the hand name not in move_group")
        
        # https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html
        move_group.set_planning_pipeline_id('ompl')
        move_group.set_planner_id('manipulator[RRTConnect]')
        move_group.set_goal_position_tolerance(0.001)
        move_group.set_goal_orientation_tolerance(0.01)
        # move_group.set_planning_time(1.0)
        # move_group.set_num_planning_attempts(10)
        # move_group.set_max_velocity_scaling_factor(0.1)
        # move_group.set_max_acceleration_scaling_factor(0.1)

        pose_goal = move_group.get_current_pose(end_effector_link).pose
        pose_goal.position.x = pose_goal.position.x
        if hand_name == "left_arm":
            pose_goal.position.x = pose_goal.position.x + 0.01
        elif hand_name == "right_arm":
            pose_goal.position.x = pose_goal.position.x - 0.01

        pose_goal.position.y = pose_goal.position.y

        pose_goal.position.z = pose_goal.position.z 
        move_group.clear_pose_targets()
        move_group.set_pose_target(pose_goal, end_effector_link)
        result, plan, fraction, others = move_group.plan()


    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        move_group.execute(plan, wait=True)
        
    
    def plan_to_pose_goals(self, hand_name):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_8_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_8_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        else:
            raise NotImplementedError("the hand name not in move_group")
        
        # https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html
        move_group.set_planning_pipeline_id('ompl')
        move_group.set_planner_id('manipulator[RRTConnect]')
        move_group.set_goal_position_tolerance(0.001)
        move_group.set_goal_orientation_tolerance(0.01)
        
        
        pose_goal = move_group.get_current_pose(end_effector_link).pose
        pose1 = copy.deepcopy(pose_goal)
        pose2 = copy.deepcopy(pose_goal)
        pose3 = copy.deepcopy(pose_goal)
        pose1.position.z = pose_goal.position.z + 0.05
        pose2.position.z = pose_goal.position.x + 0.05
        pose3.position.z = pose_goal.position.y + 0.05
        pose = [pose1, pose2, pose3]

        move_group.clear_pose_targets()
        move_group.set_pose_targets(pose, end_effector_link)
        result, plan, fraction, others = move_group.plan()

        
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        move_group.execute(plan, wait=True)
        
    def control_gripper(self, hand_name):
        if hand_name == 'left_gripper':
            move_group = self.gripper_left
        elif hand_name == 'right_gripper':
            move_group = self.gripper_right
        else:
            raise NotImplementedError("the hand name not in move_group")
        
        joint_goal = move_group.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        
        return all_close(joint_goal, current_joints, 0.01)

    def get_joint_states(self, hand_name):
        if hand_name == 'left':
            move_group = self.gripper_left
        elif hand_name == 'right':
            move_group = self.gripper_right
            
        current_joints = move_group.get_current_joint_values()
        print(f"{hand_name} current_joints: ", current_joints)
        return current_joints

    def get_ee_pose(self, hand_name):
        if hand_name == 'left':
            move_group = self.gripper_left
            end_effector_link="left_8_link"
        elif hand_name == 'right':
            move_group = self.gripper_right
            end_effector_link="right_8_link"
            
        pose_goal = move_group.get_current_pose(end_effector_link).pose
        print(f"{hand_name} pose_goal: ", pose_goal)
        return pose_goal

    def go_to_joint_state(self, move_group):
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        move_group.go(joint_goal, wait=True)
        move_group.stop()
        
        current_joints = move_group.get_current_joint_values()
        
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, move_group):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        if not success:
            raise RuntimeError("the pose goal is unable to reach.")
        
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, hand_name, scale=1):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_8_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_8_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
        else:
            raise NotImplementedError("the hand name not in move_group")
        
        
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold


    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        move_group.execute(plan, wait=True)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

        ## END_SUB_TUTORIAL



def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        while(True):
            tutorial.get_joint_states('left')
            tutorial.get_ee_pose('left')
            tutorial.get_joint_states('right')
            tutorial.get_ee_pose('right')

        # tutorial.plan_to_pose_goal("left_arm")
        # tutorial.plan_to_pose_goal("right_arm")
        # tutorial.control_gripper("left_gripper")
        # tutorial.control_gripper("right_gripper")
        # tutorial.plan_to_pose_goal("left_arm")
        # tutorial.plan_to_pose_goal("right_arm")


        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
 
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
