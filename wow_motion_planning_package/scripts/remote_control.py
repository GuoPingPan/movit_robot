from remote_control_server import SocketServer, calibrate_dual_arx, get_master_action_arx
from interface import UpRobotInterface
import numpy as np


def test():
    sock = SocketServer(ip_port=('192.168.8.143', 34556))
    robot = UpRobotInterface(fake=False )

    offset = calibrate_dual_arx(sock, )
    while True:

        action = get_master_action_arx(sock, offset)
        # print("action: ", action)
        robot.plan_to_pose_goal('left_arm', [action[0:6].tolist()], debug=False)
        robot.plan_to_pose_goal('right_arm', [action[7:13].tolist()], debug=False)
        robot.control_gripper_distance("left_arm", action[6])
        robot.control_gripper_distance("right_arm", action[13])

    sock.end()


if __name__ == '__main__':
    test()