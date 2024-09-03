#!/usr/bin/env python3

# Python 2/3 compatibility imports

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float32
import numpy as np
from utils import *
import pinocchio
from numpy.linalg import norm, solve

class UpRobotInterface(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, fake=True):
        super(UpRobotInterface, self).__init__()

        self.fake = fake

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
        
        self.move_group_left.set_planning_time(0.1)
        self.move_group_right.set_planning_time(0.1)


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

        self.left_ee_origin = [0, 0.18789, -0.2935]
        self.right_ee_origin = [0, -0.18789, -0.2935]
        

        urdf_filename = "/home/a4090/up_v2/src/wow_description/up_body_wo_dummy.urdf"
        self.model = pinocchio.buildModelFromUrdf(urdf_filename)

        self.qup = self.model.upperPositionLimit.tolist()
        self.qlo = self.model.lowerPositionLimit.tolist()
        print("model name: ", self.model.name)
        print("joint limit: ",  self.model.upperPositionLimit.tolist())
        print("joint limit: ",  self.model.lowerPositionLimit.tolist())

        self.data = self.model.createData()

        # We can get the name of the reference frame for this robot:
        # print("============ Planning frame: %s" % self.move_group_left.get_planning_frame())
        # print("============ End effector link: %s" % self.move_group_left.get_end_effector_link())
        # print("============ Planning frame: %s" % self.move_group_right.get_planning_frame())
        # print("============ End effector link: %s" % self.move_group_right.get_end_effector_link())
        # print("============ Available Planning Groups:", self.robot.get_group_names())
        # print("============ Printing robot state")

        # ============ Planning frame: base_link
        # ============ End effector link: left_7_link
        # ============ Planning frame: base_link
        # ============ End effector link: right_7_link
        # ============ Available Planning Groups: ['left_arm', 'left_arm', 'right_arm', 'right_arm']
        # ============ Printing robot state
        
        # print(self.robot.get_current_state())
        # print("")
        # import ipdb; ipdb.set_trace()

    def plan_to_pose_goal(self, hand_name, xyzrpy, use_degree=False, debug=False):

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

        for (x, y, z, r, p, yaw) in xyzrpy:
            x1 = x + origin_xyz[0]
            y1 = y + origin_xyz[1]
            z1 = z + origin_xyz[2]
            qx, qy, qz, qw = rpy_to_quaternion(r, p, yaw, use_degree)

            pose_goal_temp = copy.deepcopy(pose_goal)
            pose_goal_temp.position.x = x1
            pose_goal_temp.position.y = y1
            pose_goal_temp.position.z = z1

            pose_goal_temp.orientation.x = qx
            pose_goal_temp.orientation.y = qy
            pose_goal_temp.orientation.z = qz
            pose_goal_temp.orientation.w = qw

            pose_goals.append(pose_goal_temp)
            

            if debug:
                r, p, yaw = quaternion_to_rpy([qx, qy, qz, qw])
                
                print(f"============ [{hand_name}] ============")
                print(f" pose_goal: ", [x, y, z])
                print(f" rpy: ", [r, p, yaw])
                print("\n")

        move_group.clear_pose_targets()
        move_group.set_pose_targets(pose_goals, end_effector_link)
        success, plan, fraction, others = move_group.plan()

        if not success:
            print(f"[Fail] {hand_name} IK & planning has not solution.")
            return None

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        
        if self.fake:
            move_group.execute(plan, wait=True)
        else:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)

            # Publish
            display_trajectory_publisher.publish(display_trajectory)

        return plan
        
    def plan_to_joint(self, hand_name, joint_values):

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
            joint_goal[0] = joint_values[0]
            joint_goal[1] = joint_values[1]
            joint_goal[2] = joint_values[2]
            joint_goal[3] = joint_values[3]
            joint_goal[4] = joint_values[4]
            joint_goal[5] = joint_values[5]
            joint_goal[6] = joint_values[6]
            # joint_goal[7] = joint_values[7] * self.joint_control_pos_minus[7]
        elif hand_name == "right_arm":
            joint_goal[0] = joint_values[0]
            joint_goal[1] = joint_values[1]
            joint_goal[2] = joint_values[2]
            joint_goal[3] = joint_values[3]
            joint_goal[4] = joint_values[4]
            joint_goal[5] = joint_values[5]
            joint_goal[6] = joint_values[6]
            # joint_goal[7] = joint_values[7] * self.joint_control_pos_minus[15]

        move_group.set_joint_value_target(joint_goal)
        success, plan, fraction, others = move_group.plan()

        if not success:
            print("[Fail] IK & planning has not solution.")
            return None

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )

        if self.fake:
            move_group.execute(plan, wait=True)

        else:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)

            # Publish
            display_trajectory_publisher.publish(display_trajectory)

        return plan

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
        # print(pose_goal)

        for (x, y, z, r, p, yaw) in xyzrpy:
            x += origin_xyz[0]
            y += origin_xyz[1]
            z += origin_xyz[2]
            qx, qy, qz, qw = rpy_to_quaternion(r, p, yaw)

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

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )

        if self.fake:
            move_group.execute(plan, wait=True)
        else:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)

            display_trajectory_publisher.publish(display_trajectory)

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan

    def get_joint_states(self, hand_name):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            
        current_joints = move_group.get_current_joint_values()
        print(f"============ [{hand_name}] ============")
        print(f" current_joints: ", current_joints)

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

    def control_gripper_bool(self, hand_name, open):
        gripper_rad = 0
        if hand_name == 'left_arm':
            gripper_pub = self.left_gripper_pub
            if open:
                gripper_rad = -1 * self.joint_control_pos_minus[7]
                # print(f"open {hand_name} gripper")

        elif hand_name == 'right_arm':
            gripper_pub = self.right_gripper_pub
            if open:
                gripper_rad = -1 * self.joint_control_pos_minus[15]
                # print(f"open {hand_name} gripper")
        

        gripper_pub.publish(gripper_rad)
        return gripper_rad

    def control_gripper_distance(self, hand_name, open):
        gripper_rad = 0
        if hand_name == 'left_arm':
            gripper_pub = self.left_gripper_pub
            if open:
                gripper_rad = open * self.joint_control_pos_minus[7]
                # print(f"open {hand_name} gripper")

        elif hand_name == 'right_arm':
            gripper_pub = self.right_gripper_pub
            if open:
                gripper_rad = open * self.joint_control_pos_minus[15]
                # print(f"open {hand_name} gripper")
        

        gripper_pub.publish(gripper_rad)
        return gripper_rad


    def ik_control_f(self, hand_name, xyzrpy):
            if hand_name == 'left_arm':
                move_group = self.move_group_left
                end_effector_link="left_7_link"
                display_trajectory_publisher = self.left_display_trajectory_publisher
                cj = self.get_joint_states(hand_name)
                cj = np.array([*cj] + [0]*9)

            elif hand_name == 'right_arm':
                move_group = self.move_group_right
                end_effector_link="right_7_link"
                display_trajectory_publisher = self.right_display_trajectory_publisher
                cj = self.get_joint_states(hand_name)
                cj = np.array([0]*8 + [*cj] + [0])

            else:
                raise NotImplementedError("the hand name not in move_group")

            # 输出每个帧的 ID 和名称
            for frame_id in range(self.model.nframes):
                frame = self.model.frames[frame_id]
                print(f"Frame ID: {frame_id}, Name: {frame.name}")


            eps = 5e-3
            IT_MAX = 10000
            DT = 1e-1
            damp = 1e-12
            JOINT_ID = 18
            INTERPOLATE = 2


            qup = self.model.upperPositionLimit.tolist()
            qlow = self.model.lowerPositionLimit.tolist()
            qup = np.array(qup)
            qlow = np.array(qlow)
            random_samples = np.random.rand(len(qlow))
            
            random_arrays = [qlow + (qup - qlow) * random_samples]
            zero_padding = [np.zeros(self.model.nq - len(qup))]
            cj = np.concatenate(random_arrays + zero_padding)


            pinocchio.forwardKinematics(self.model, self.data, cj)
            pinocchio.updateFramePlacements(self.model, self.data)

            if self.fake:
                self.plan_to_joint(hand_name, cj[:8])

            # origin_xyz = self.data.oMf[JOINT_ID].translation 
            origin_xyz = self.data.oMf[JOINT_ID]
            # + np.array([0, 0, -0.1])

            print("start: ", self.data.oMf[JOINT_ID])

            print(origin_xyz)
            print(self.data.oMf[JOINT_ID])
            print(self.data.oMf[JOINT_ID+1])
            print(self.data.oMf[JOINT_ID+2])
            # return
            print(len(self.data.oMf))
            # return
            # for joint_id in range(len(self.data.oMf)):
            #     # joint = self.model.joints[joint_id]
                
            #     # 获取关节名称
            #     # joint_name = self.model.names[joint_id]
                
            #     # 获取关节的姿态
            #     oMf = self.data.oMf[joint_id]
            #     position = oMf.translation
            #     orientation = oMf.rotation
                
            #     # print(f"Joint ID: {joint_id}, Name: {joint_name}")
            #     print(f"  Position: {position}")
            #     print(f"  Orientation (Rotation Matrix): \n{orientation}")
            #     print()

            
            # pos = np.array(xyzrpy[:3]) + origin_xyz
            # rot = rpy_to_rotation_matrix(*xyzrpy[3:])
            # oMdes = pinocchio.SE3(rot, pos)

            import copy
            oMdes = copy.deepcopy(origin_xyz)
            oMdes.translation += np.array([0, 0, 0.05])


            pinocchio.forwardKinematics(self.model, self.data, cj)
            pinocchio.updateFramePlacements(self.model, self.data)
            
            coMi = self.data.oMf[JOINT_ID]
            
            interp_seq = []
            for i in range(1, INTERPOLATE):
                alpha = i * 1
                interp_seq.append(pinocchio.SE3.Interpolate(coMi, oMdes, alpha))

            for target in interp_seq:
                target.rotation[target.rotation < 1e-12] = 0
                # print("target: ", target)

                while True:
                    pinocchio.forwardKinematics(self.model, self.data, cj)
                    pinocchio.updateFramePlacements(self.model, self.data)
                    
                    # cp = copy.deepcopy( self.data.oMf[JOINT_ID])
                    # rot = cp.rotation
                    # delta = cp.rotation @ np.array([0, 0, -0.1])
                    # cp.translation += delta
                    # print("=" * 10)
                    # print(self.data.oMf[JOINT_ID])
                    # print(self.data.oMf[JOINT_ID+1])
                    # print(self.data.oMf[JOINT_ID+2])
                    # print(cp)
                    # import ipdb; ipdb.set_trace()
                    
                    # iMd = cp.actInv(target)
                    iMd = self.data.oMf[JOINT_ID].actInv(target)
                    err = pinocchio.log(iMd).vector  # in joint frame

                    if norm(err) < eps:
                        print("success")
                        success = True
                        break
                    if i >= IT_MAX:
                        print("unsuccessful")
                        success = False
                        break
                    # J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame
                    # J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame

                    J = pinocchio.computeFrameJacobian(self.model, self.data, cj, JOINT_ID)

                    # print(help(pinocchio.computeJointJacobian))
                    J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
                    v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
                    cj = pinocchio.integrate(self.model, cj, v * DT)
                    cj = np.clip(cj, self.qlo, self.qup)
                    # if not i % 10:
                    #     print("%d: error = %s" % (i, err.T))
                    i += 1


                for i in range(len(cj)):
                    if cj[i] > self.qup[i] or cj[i] < self.qlo[i]:
                        print(i, cj[i], self.qup[i], self.qlo[i])
                        raise ValueError
                print(cj)
                if self.fake:
                    self.plan_to_joint(hand_name, cj[:8])
            pinocchio.forwardKinematics(self.model, self.data, cj)
            print("after: ", self.data.oMf[JOINT_ID])


    def ik_control_ff(self, hand_name, xyzrpy):
            if hand_name == 'left_arm':
                move_group = self.move_group_left
                end_effector_link="left_7_link"
                display_trajectory_publisher = self.left_display_trajectory_publisher
                cj = self.get_joint_states(hand_name)
                cj = np.array([*cj] + [0]*9)

            elif hand_name == 'right_arm':
                move_group = self.move_group_right
                end_effector_link="right_7_link"
                display_trajectory_publisher = self.right_display_trajectory_publisher
                cj = self.get_joint_states(hand_name)
                cj = np.array([0]*8 + [*cj] + [0])

            else:
                raise NotImplementedError("the hand name not in move_group")

            # 输出每个帧的 ID 和名称
            for frame_id in range(self.model.nframes):
                frame = self.model.frames[frame_id]
                print(f"Frame ID: {frame_id}, Name: {frame.name}")
                
            for joint_id in range(self.model.njoints):
                joint = self.model.joints[joint_id]
                
                # 获取关节名称
                joint_name = self.model.names[joint_id]
                
                # 获取关节的姿态
                oMi = self.data.oMi[joint_id]
                position = oMi.translation
                orientation = oMi.rotation
                
                print(f"Joint ID: {joint_id}, Name: {joint_name}")
                # print(f"  Position: {position}")
                # print(f"  Orientation (Rotation Matrix): \n{orientation}")
                print()


            eps = 5e-3
            IT_MAX = 10000
            DT = 1e-2
            damp = 1e-12
            JOINT_ID = 16
            INTERPOLATE = 2


            qup = self.model.upperPositionLimit.tolist()
            qlow = self.model.lowerPositionLimit.tolist()
            qup = np.array(qup)
            qlow = np.array(qlow)
            random_samples = np.random.rand(len(qlow))
            
            cj = np.zeros(16)


            pinocchio.forwardKinematics(self.model, self.data, cj)
            pinocchio.updateFramePlacements(self.model, self.data)


            origin_xyz = self.data.oMf[JOINT_ID].translation 
            # origin_xyz = self.data.oMf[JOINT_ID]
            # + np.array([0, 0, -0.1])


            pos = np.array(xyzrpy[:3]) + origin_xyz
            rot = rpy_to_rotation_matrix(*xyzrpy[3:])
            oMdes = pinocchio.SE3(rot, pos)
            
            coMi = self.data.oMf[JOINT_ID]
            print("start: ", coMi)
            
            interp_seq = []
            for i in range(1, INTERPOLATE):
                alpha = i * 1
                interp_seq.append(pinocchio.SE3.Interpolate(coMi, oMdes, alpha))

            for target in interp_seq:
                # target.rotation[target.rotation < 1e-12] = 0
                print("target: ", target)

                while True:
                    pinocchio.forwardKinematics(self.model, self.data, cj)
                    pinocchio.updateFramePlacements(self.model, self.data)
                    
                    # cp = copy.deepcopy( self.data.oMf[JOINT_ID])
                    # rot = cp.rotation
                    # delta = cp.rotation @ np.array([0, 0, -0.1])
                    # cp.translation += delta
                    # print("=" * 10)
                    # print(self.data.oMf[JOINT_ID])
                    # print(self.data.oMf[JOINT_ID+1])
                    # print(self.data.oMf[JOINT_ID+2])
                    # print(cp)
                    # import ipdb; ipdb.set_trace()
                    
                    # iMd = cp.actInv(target)
                    iMd = self.data.oMf[JOINT_ID].actInv(target)
                    err = pinocchio.log(iMd).vector  # in joint frame

                    if norm(err) < eps:
                        print("success")
                        success = True
                        break
                    if i >= IT_MAX:
                        print(i)
                        print("unsuccessful")
                        success = False
                        break
                    # J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame
                    # J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame
                    J = pinocchio.computeFrameJacobian(self.model, self.data, cj, JOINT_ID)
                    # print(help(pinocchio.computeJointJacobian))
                    J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
                    v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
                    cj = pinocchio.integrate(self.model, cj, v * DT)
                    cj = np.clip(cj, self.qlo, self.qup)
                    # if not i % 10:
                    #     print("%d: error = %s" % (i, err.T))
                    i += 1

                
                print(cj)
                for i in range(len(cj)):
                    if cj[i] > self.qup[i] or cj[i] < self.qlo[i]:
                        print(i, cj[i], self.qup[i], self.qlo[i])
                        raise ValueError
                if self.fake:
                    self.plan_to_joint(hand_name, cj[:8])
            pinocchio.forwardKinematics(self.model, self.data, cj)
            pinocchio.updateFramePlacements(self.model, self.data)
            print("after: ", JOINT_ID, self.data.oMf[JOINT_ID])
            print("after: ", JOINT_ID-1, self.data.oMf[JOINT_ID-1])
            print(len(self.data.oMi), len(self.data.iMf))
            print(self.data.oMi[0], self.data.iMf[0])
            
            print("oMi: ", JOINT_ID, self.data.oMi[7])
            print("iMf: ", JOINT_ID, self.data.iMf[7])

            for i in range(len(self.data.oMi)):
                print()
                print(i)
                print(self.data.oMi[i], self.data.iMf[i])
                if i > 0:
                    print(cj[i-1])


    def ik_control(self, hand_name, xyzrpy):
        if hand_name == 'left_arm':
            move_group = self.move_group_left
            end_effector_link="left_7_link"
            display_trajectory_publisher = self.left_display_trajectory_publisher
            cj = self.get_joint_states(hand_name)
            cj = np.array([*cj] + [0]*9)

        elif hand_name == 'right_arm':
            move_group = self.move_group_right
            end_effector_link="right_7_link"
            display_trajectory_publisher = self.right_display_trajectory_publisher
            cj = self.get_joint_states(hand_name)
            cj = np.array([0]*8 + [*cj] + [0])

        else:
            raise NotImplementedError("the hand name not in move_group")

        # 输出每个帧的 ID 和名称
        for frame_id in range(self.model.nframes):
            frame = self.model.frames[frame_id]
            print(f"Frame ID: {frame_id}, Name: {frame.name}")

 

        eps = 5e-7
        IT_MAX = 2000
        DT = 1e-1
        damp = 1e-12
        JOINT_ID = 7
        INTERPOLATE = 2

        pinocchio.forwardKinematics(self.model, self.data, np.zeros_like(cj))
        pinocchio.updateFramePlacements(self.model, self.data)
        origin_xyz = self.data.oMi[JOINT_ID].translation
        print("start: ", self.data.oMi[JOINT_ID])

        for joint_id in range(self.model.njoints):
            joint = self.model.joints[joint_id]
            
            # 获取关节名称
            joint_name = self.model.names[joint_id]
            
            # 获取关节的姿态
            oMi = self.data.oMi[joint_id]
            position = oMi.translation
            orientation = oMi.rotation
            
            print(f"Joint ID: {joint_id}, Name: {joint_name}")
            print(f"  Position: {position}")
            print(f"  Orientation (Rotation Matrix): \n{orientation}")
            print()

        pos = np.array(xyzrpy[:3]) + origin_xyz
        rot = rpy_to_rotation_matrix(*xyzrpy[3:])
        oMdes = pinocchio.SE3(np.eye(3), pos)

        pinocchio.forwardKinematics(self.model, self.data, cj)
        pinocchio.updateFramePlacements(self.model, self.data)
        coMi = self.data.oMi[JOINT_ID]
        interp_seq = []
        for i in range(1, INTERPOLATE):
            alpha = i * 1
            interp_seq.append(pinocchio.SE3.Interpolate(coMi, oMdes, alpha))

        for target in interp_seq:
            target.rotation[target.rotation < 1e-12] = 0
            print("target: ", target)

            while True:
                pinocchio.forwardKinematics(self.model, self.data, cj)
                pinocchio.updateFramePlacements(self.model, self.data)
                iMd = self.data.oMi[JOINT_ID].actInv(target)
                err = pinocchio.log(iMd).vector  # in joint frame

                if norm(err) < eps:
                    print("success")
                    success = True
                    break
                if i >= IT_MAX:
                    print("unsuccessful")
                    success = False
                    break
                J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame
                # J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame

                # J = pinocchio.computeFrameJacobian(self.model, self.data, cj, 19, pinocchio.LOCAL_WORLD_ALIGNED)

                # print(help(pinocchio.computeJointJacobian))
                J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
                v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
                cj = pinocchio.integrate(self.model, cj, v * DT)
                # if not i % 10:
                #     print("%d: error = %s" % (i, err.T))
                i += 1

            cj = np.clip(cj, self.qlo, self.qup)

            for i in range(len(cj)):
                if cj[i] > self.qup[i] or cj[i] < self.qlo[i]:
                    print(i, cj[i], self.qup[i], self.qlo[i])
                    raise ValueError
            print(cj)

        # oMdes = pinocchio.SE3(rot, pos)
        # pinocchio.forwardKinematics(self.model, self.data, cj)
        # pinocchio.updateFramePlacements(self.model, self.data)
        # coMi = self.data.oMi[JOINT_ID]
        # interp_seq = []
        # for i in range(1, INTERPOLATE):
        #     alpha = i * 1
        #     interp_seq.append(pinocchio.SE3.Interpolate(coMi, oMdes, alpha))

        # for target in interp_seq:
        #     target.rotation[target.rotation < 1e-12] = 0
        #     print("target: ", target)

        #     while True:
        #         pinocchio.forwardKinematics(self.model, self.data, cj)
        #         pinocchio.updateFramePlacements(self.model, self.data)
        #         iMd = self.data.oMi[JOINT_ID] * self.data.oMi[JOINT_ID-1]
        #         iMd = iMd.actInv(target)
        #         err = pinocchio.log(iMd).vector  # in joint frame

        #         if norm(err) < eps:
        #             print("success")
        #             success = True
        #             break
        #         if i >= IT_MAX:
        #             print("unsuccessful")
        #             success = False
        #             break
        #         J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame
        #         # J = pinocchio.computeJointJacobian(self.model, self.data, cj, JOINT_ID)  # in joint frame

        #         # J = pinocchio.computeFrameJacobian(self.model, self.data, cj, 19, pinocchio.LOCAL_WORLD_ALIGNED)

        #         # print(help(pinocchio.computeJointJacobian))
        #         J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        #         v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        #         cj = pinocchio.integrate(self.model, cj, v * DT)
        #         # if not i % 10:
        #         #     print("%d: error = %s" % (i, err.T))
        #         i += 1

        #     cj = np.clip(cj, self.qlo, self.qup)

        #     for i in range(len(cj)):
        #         if cj[i] > self.qup[i] or cj[i] < self.qlo[i]:
        #             print(i, cj[i], self.qup[i], self.qlo[i])
        #             raise ValueError
        #     print(cj)

        # pinocchio.forwardKinematics(self.model, self.data, cj)
        # pinocchio.updateFramePlacements(self.model, self.data)
        print("after: ", self.data.oMi[JOINT_ID])
        if self.fake:
            self.plan_to_joint(hand_name, cj[:7])

        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 收集所有关节位置和姿态
        positions = []
        x_axes = []
        y_axes = []
        z_axes = []

        for joint_id in range(self.model.njoints):
            joint = self.model.joints[joint_id]
            oMi = self.data.oMi[joint_id]
            position = oMi.translation
            rotation = oMi.rotation
            assert is_orthogonal(rotation)
            
            # 单位基向量
            unit_x = np.array([1, 0, 0])
            unit_y = np.array([0, 1, 0])
            unit_z = np.array([0, 0, 1])

            # 旋转后的基向量
            x_axis = rotation @ unit_x
            y_axis = rotation @ unit_y
            z_axis = rotation @ unit_z

            positions.append(position)
            x_axes.append(x_axis)
            y_axes.append(y_axis)
            z_axes.append(z_axis)

        # 转换为numpy数组
        positions = np.array(positions)
        x_axes = np.array(x_axes)
        y_axes = np.array(y_axes)
        z_axes = np.array(z_axes)

        # 绘制关节位置
        ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], c='r', marker='o')

        # 绘制坐标轴
        for i in range(len(positions)):
            # 绘制X轴 (红色)
            ax.quiver(positions[i, 0], positions[i, 1], positions[i, 2],
                    x_axes[i, 0], x_axes[i, 1], x_axes[i, 2], color='r', length=0.1)
            # 绘制Y轴 (绿色)
            ax.quiver(positions[i, 0], positions[i, 1], positions[i, 2],
                    y_axes[i, 0], y_axes[i, 1], y_axes[i, 2], color='g', length=0.1)
            # 绘制Z轴 (蓝色)
            ax.quiver(positions[i, 0], positions[i, 1], positions[i, 2],
                    z_axes[i, 0], z_axes[i, 1], z_axes[i, 2], color='b', length=0.1)

        # 设置坐标轴标签
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(-0.5, 0.5)

        plt.show()


def is_orthogonal(matrix, tol=1e-10):
    """
    检查给定的矩阵是否为正交矩阵。
    
    参数:
    - matrix (np.ndarray): 需要检查的矩阵。
    - tol (float): 容差值，用于判断矩阵是否接近单位矩阵。默认为1e-10。

    返回:
    - bool: 如果矩阵是正交的，则返回True，否则返回False。
    """
    # 检查矩阵是否为方阵
    if matrix.shape[0] != matrix.shape[1]:
        return False

    # 计算矩阵与其转置的乘积
    identity = np.eye(matrix.shape[0])
    product = matrix @ matrix.T
    
    # 检查结果是否接近单位矩阵
    return np.allclose(product, identity, atol=tol)


def main():
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    interface = UpRobotInterface(fake=True)

    # while not rospy.is_shutdown():
    #     interface.control_gripper("left_arm", open=True)
    #     interface.control_gripper("right_arm", open=True)
    
    # xyzrpy = [0.1, 0.0, 0.1, np.deg2rad(0), np.deg2rad(-0), np.deg2rad(20)]
    # interface.ik_control_ff("left_arm", xyzrpy,)
    
    interface.plan_to_joint("left_arm", [0.563249, -0.039102, 0.207713, 0.371748, 0.075453, -0.76095, -0.104598])
    interface.plan_to_joint("left_arm", [0.6173728501691044, -0.03822271158274356, 1.5721279232089496, 0.11607352552681544, 1.6995437961872457, -1.0717620868584148, 0.09699593086455081])
    
    
    
    
    # interface.plan_to_joint("left_arm", [0.851251083153424, -0.09466740196669636, 0.17362340523537345, -0.8688228066698169, -0.0965501768520479, -1.7108809890471852, -0.190142196166166])
    
    # interface.plan_to_joint("left_arm", [0.563249, -0.039102, 0.207713, 0.371748, 0.075453, -0.76095, -0.104598])
    
    # interface.plan_to_joint("left_arm", [0.8153805603791195, -0.02019785462660401, 1.5600560869000861, 0.11553498362928045, 1.3567447540402573, -0.8667175240124863, -0.1501173425799801])
    
    # interface.plan_to_joint("left_arm", [0.48418162081424826, -0.04017831707729767, 0.27618261766751717, 0.7283072413443811, 0.18876729709405762, -0.4970195220718506, -0.16259611863391782])
    
    
    # interface.plan_to_joint("left_arm", [0.46411358737834535, 0.15147885443502263, 0.3466842456756618, 1.5723551288183022, 0.2690576694298714, 0.2679099489855924, -0.3299034065095933])
    # interface.plan_to_joint("left_arm", [0.5616489126432401, 0.015336244921695196, 1.5596096435876574, 0.1163632206091558, 1.352927849971358, -1.117440407753923, -0.18168749200996293])

    # interface.get_ee_pose("left_arm")

    # xyzrpy = [[0.1, 0, 0.1, 0, 20, 0]]
    # interface.plan_to_pose_goal("left_arm", xyzrpy, use_degree=True)
    # # interface.plan_to_pose_goal("right_arm", xyzrpy)
    
    # xyzrpy = [[0, 0, 0.0, 0, 0, 0]]
    # interface.plan_cartesian_path("right_arm", xyzrpy)

    # xyzrpy = [[0.1, 0, 0.1, 0, 0, 0]]
    # interface.plan_to_pose_goal("right_arm", xyzrpy)
    
    # xyzrpy = [[0, 0, 0.0, 0, 0, 0]]
    # interface.plan_to_pose_goal("right_arm", xyzrpy)

if __name__ == "__main__":
    main()
