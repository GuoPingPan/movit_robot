#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

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
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

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
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        
        # ============ Planning frame: base_link
        # ============ End effector link: left_7_link
        # ============ Planning frame: base_link
        # ============ End effector link: right_7_link
        # ============ Available Planning Groups: ['left_arm', 'left_gripper', 'right_arm', 'right_gripper']
        # ============ Printing robot state
        
        # We can get the name of the reference frame for this robot:
        print("============ Planning frame: %s" % self.move_group_left.get_planning_frame())
        print("============ End effector link: %s" % self.move_group_left.get_end_effector_link())
        print("============ Planning frame: %s" % self.move_group_right.get_planning_frame())
        print("============ End effector link: %s" % self.move_group_right.get_end_effector_link())
        print("============ Available Planning Groups:", self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.scene = scene
        self.display_trajectory_publisher = display_trajectory_publisher

    def plan_to_pose_goal(self, move_group, end_effector_link="left_7_link"):
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
        pose_goal.position.y = pose_goal.position.y 
        pose_goal.position.z = pose_goal.position.z + 0.05
        move_group.clear_pose_targets()
        move_group.set_pose_target(pose_goal, end_effector_link)
        result, plan, fraction, others = move_group.plan()

        print(type(pose_goal))
        print(result)
        print(plan)
        print(fraction)
        print(others)

        self.display_trajectory(plan)
        self.execute_plan(move_group, plan)
        
    def plan_to_pose_goals(self, move_group, end_effector_link="left_7_link"):
        # https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html
        move_group.set_planning_pipeline_id('ompl')
        move_group.set_planner_id('manipulator[RRTConnect]')
        # move_group.set_planning_time(1.0)
        # move_group.set_num_planning_attempts(10)
        # move_group.set_max_velocity_scaling_factor(0.1)
        # move_group.set_max_acceleration_scaling_factor(0.1)

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

        print(result)
        print(plan)
        print(fraction)
        print(others)

        self.display_trajectory(plan)
        self.execute_plan(move_group, plan)
        
    def control_gripper(self, move_group):
        joint_goal = move_group.get_current_joint_values()
        
        joint_goal[0] = 0
        joint_goal[1] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        
        return all_close(joint_goal, current_joints, 0.01)

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

    def plan_cartesian_path(self, move_group, scale=1):
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

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

    
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        import ipdb; ipdb.set_trace()
        pass
        
        ## END_SUB_TUTORIAL

    def execute_plan(self, move_group, plan):
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, frame_id, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = frame_id
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, grasping_group, eef_link, box_name, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        scene = self.scene

        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, eef_link, box_name, timeout=4):
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, box_name, timeout=4):
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


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

        tutorial.plan_to_pose_goals(tutorial.move_group_left)
        tutorial.plan_to_pose_goal(tutorial.move_group_left)
        tutorial.control_gripper(tutorial.gripper_left)

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        tutorial.go_to_joint_state(tutorial.move_group_left)
        tutorial.go_to_joint_state(tutorial.move_group_right)

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal(tutorial.move_group_left)
        tutorial.go_to_pose_goal(tutorial.move_group_right)

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        left_cartesian_plan, fraction = tutorial.plan_cartesian_path(tutorial.move_group_left)
        right_cartesian_plan, fraction = tutorial.plan_cartesian_path(tutorial.move_group_right)

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(left_cartesian_plan)
        tutorial.display_trajectory(right_cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(tutorial.move_group_left, left_cartesian_plan)
        tutorial.execute_plan(tutorial.move_group_right, right_cartesian_plan)

        input("============ Press `Enter` to execute right path to left arm ...")
        tutorial.execute_plan(tutorial.move_group_left, right_cartesian_plan)


        input("============ Press `Enter` to add a box to the planning scene ...")
        tutorial.add_box("left_8_link", "box1")
        tutorial.add_box("right_8_link", "box2")

        input("============ Press `Enter` to attach a Box to the Panda robot ...")
        tutorial.attach_box("left_gripper", "left_8_link", "box1")
        tutorial.attach_box("right_gripper", "right_8_link", "box2")

        input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
        cartesian_plan, fraction = tutorial.plan_cartesian_path(tutorial.move_group_left, scale=-1)
        tutorial.execute_plan(tutorial.move_group_left, cartesian_plan)
        tutorial.execute_plan(tutorial.move_group_right, cartesian_plan)
        
        input("============ Press `Enter` to detach the box from the Panda robot ...")
        tutorial.detach_box("left_8_link", "box1")
        tutorial.detach_box("right_8_link", "box2")

        input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
        tutorial.remove_box()

        print("============ Python tutorial demo complete!")
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
