import socket
import json
import time
import numpy as np

def get_master_action_arx(sock, offset):
    sock.request()
    action_ = sock.receive()
    action = master_to_arm_arx(action_, offset)
    return action


def master_to_arm_arx(action, offset):
    roll = -action[2]
    pitch = -action[0]
    yaw = action[1]
    x = -action[5]
    y = -action[3]
    z = action[4]
    gripper = action[6]

    roll1 = -action[9]
    pitch1 = -action[7]
    yaw1 = action[8]
    x1 = -action[12]
    y1 = -action[10]
    z1 = action[11]
    gripper1 = action[13]
    master_action = np.array([x, y, z, roll, pitch, yaw, -2 * gripper,
                              x1, y1, z1, roll1, pitch1, yaw1, -2 * gripper1])
    return master_action + offset


def calibrate_dual_arx(sock, start_pose=None):
    print("start calibrating,don't move the controller!")
    count = np.zeros(14)
    for i in range(50):
        sock.request()
        action_ = sock.receive()
        action_ = np.array(action_)
        count += action_
    print("calibration done")
    count = count / 50

    roll1 = -count[2]
    pitch1 = -count[0]
    yaw1 = count[1]
    x1 = -count[5]
    y1 = -count[3]
    z1 = count[4]

    roll2 = -count[9]
    pitch2 = -count[7]
    yaw2 = count[8]
    x2 = -count[12]
    y2 = -count[10]
    z2 = count[11]


    if start_pose is None:
        return np.array([-x1, -y1, -z1, -roll1, -pitch1, -yaw1,  0.,
                         -x2, -y2, -z2, -roll2, -pitch2, -yaw2, 0.])
    else:
        return np.array([-x1, -y1, -z1, -roll1, -pitch1, -yaw1,  0.,
                         -x2, -y2, -z2, -roll2, -pitch2, -yaw2, 0.]) + start_pose


class SocketServer:
    def __init__(self, ip_port=('192.168.8.143', 34561)):
        self.ip_port = ip_port
        self.server, self.sk, self.addr = self.setup()

    def setup(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(self.ip_port)
        server.listen(1)
        sk, addr = server.accept()

        return server, sk, addr

    def request(self):
        req = "1"
        self.sk.send(req.encode())

    def receive(self):
        data_ = self.sk.recv(1024)
        data = json.loads(data_)
        return data

    def end(self):
        req = '0'
        self.sk.send(req.encode())
        self.server.close()


def test():
    sock = SocketServer()
    count = 0
    while True:
        for i in range(100):
            time1 = time.time()
            sock.request()
            data = sock.receive()
            time2 = time.time()
            print(data)
            count += time2 - time1
        break
    print(count/100)
    sock.end()


if __name__ == "__main__":
    test()