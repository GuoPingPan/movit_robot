import math
import numpy as np


def rpy_to_rotation_matrix(roll, pitch, yaw, use_degree=False):
    """
    Convert Roll, Pitch, Yaw (RPY) angles to a rotation matrix.
    
    Parameters:
    roll (float): Roll angle in radians.
    pitch (float): Pitch angle in radians.
    yaw (float): Yaw angle in radians.
    
    Returns:
    numpy.ndarray: A 3x3 rotation matrix.
    """
    # Rotation matrix around the X axis (roll)
    if use_degree:
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)


    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Rotation matrix around the Y axis (pitch)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation matrix around the Z axis (yaw)
    R_z = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix (R = R_z * R_y * R_x)
    rotation_matrix = R_z @ R_y @ R_x
    
    return rotation_matrix

def rotation_matrix_to_rpy(rotation_matrix):
    """
    Convert a rotation matrix to Roll, Pitch, Yaw (RPY) angles.
    
    Parameters:
    rotation_matrix (numpy.ndarray): A 3x3 rotation matrix.
    
    Returns:
    tuple: Roll, Pitch, Yaw angles in radians.
    """
    # Ensure the input is a 3x3 matrix
    rotation_matrix = np.array(rotation_matrix)
    if rotation_matrix.shape != (3, 3):
        raise ValueError("The input must be a 3x3 rotation matrix.")
    
    # Extract the elements of the rotation matrix
    r11, r12, r13 = rotation_matrix[0]
    r21, r22, r23 = rotation_matrix[1]
    r31, r32, r33 = rotation_matrix[2]
    
    # Calculate the pitch angle
    if r31 != 1 and r31 != -1:
        pitch = -np.arcsin(r31)
        roll = np.arctan2(r32 / np.cos(pitch), r33 / np.cos(pitch))
        yaw = np.arctan2(r21 / np.cos(pitch), r11 / np.cos(pitch))
    else:
        # gimbal lock case
        yaw = 0
        if r31 == -1:
            pitch = np.pi / 2
            roll = yaw + np.arctan2(r12, r13)
        else:
            pitch = -np.pi / 2
            roll = -yaw + np.arctan2(-r12, -r13)
    
    return roll, pitch, yaw


def rpy_to_quaternion(roll, pitch, yaw, use_degree=False):
    if use_degree:
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
