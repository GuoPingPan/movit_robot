import rospy
from std_msg.msg import Float32, Float32MultiArray
import pinocchio
from numpy.linalg import norm, solve
import numpy as np
from utils import *


class UpRobotInterface(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self, fake=True):
        super(UpRobotInterface, self).__init__()

        self.fake = fake

        
        self.move_group_left.set_planning_time(0.1)
        self.move_group_right.set_planning_time(0.1)

        self.left_arm_pub = rospy.Publisher("arm_left", Float32MultiArray, queue_size=1)
        self.right_arm_pub = rospy.Publisher("arm_right", Float32MultiArray, queue_size=1)

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
            
        urdf_filename = "/home/a4090/up_v1/src/wow_description/up_body.urdf"
        self.model = pinocchio.buildModelFromUrdf(urdf_filename)

        self.qup = self.model.upperPositionLimit.tolist()
        self.qlo = self.model.lowerPositionLimit.tolist()
        print("model name: ", self.model.name)
        print("joint limit: ",  self.model.upperPositionLimit.tolist())
        print("joint limit: ",  self.model.lowerPositionLimit.tolist())

        self.data = self.model.createData()

        


