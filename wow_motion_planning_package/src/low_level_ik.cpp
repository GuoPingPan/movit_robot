#include "EtherCAT_Motor.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// 控制数据
float left_joint_control[7] = {0};
float right_joint_control[7] = {0};
float left_gripper_control = 0;
float right_gripper_control = 0;

// Float32array 回调函数
void LeftControlCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // 填充左侧关节控制数据
    for (int i = 0; i < 7; ++i) {
        left_joint_control[i] = msg->data[i];
    }
}

// Float32array 回调函数
void RightControlCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // 填充右侧关节控制数据
    for (int i = 8; i < 15; ++i) {
        right_joint_control[i] = msg->data[i];
    }
}

// Float32 回调函数
void LeftGripperCallback(const std_msgs::Float32::ConstPtr& msg) {
    left_gripper_control = msg->data;
}

// Float32 回调函数
void RightGripperCallback(const std_msgs::Float32::ConstPtr& msg) {
    right_gripper_control = msg->data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_planning");
    ros::NodeHandle nh;
    
    // 关节状态发布者
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 订阅者
    ros::Subscriber left_arm_sub = nh.subscribe("/left_joint_control", 1, LeftControlCallback);
    ros::Subscriber right_arm_sub = nh.subscribe("/right_joint_control", 1, RightControlCallback);
    ros::Subscriber left_gripper_sub = nh.subscribe("/left_gripper_control", 1, LeftGripperCallback);
    ros::Subscriber right_gripper_sub = nh.subscribe("/right_gripper_control", 1, RightGripperCallback);

    ros::Rate loop_rate(500);

    // 关节名称和ID的映射
    std::string joint_names[16] = {
        "left_1_joint", "left_2_joint", "left_3_joint", "left_4_joint",
        "left_5_joint", "left_6_joint", "left_7_joint", "left_8_joint",
        "right_1_joint", "right_2_joint", "right_3_joint", "right_4_joint",
        "right_5_joint", "right_6_joint", "right_7_joint", "right_8_joint"
    };

    float joint_pos_minus[16] = {
        -1, -1, -1, -1,
        1, 1, 1, 1,
        -1, 1, -1, -1,
        1, 1, 1, -1,
    };

    int joint_ids[16] = {
        7, 6, 5, 4,
        3, 2, 1, 0,
        8, 9, 10, 11,
        12, 13, 14, 15
    };

    int ret = EtherCAT_Init(DC_Sync, 5000);

    if (ret == FALSE) {
        ROS_ERROR("EtherCAT initialization failed!");
        return 0;
    }

    // 初始化电机控制方式
    for (int i = 0; i < 16; ++i) {
        EtherCAT_Motor[joint_ids[i]].control.Control_Mode = POS_CONTROL;
        EtherCAT_Motor[joint_ids[i]].control.KP = 0;
        EtherCAT_Motor[joint_ids[i]].control.KD = 0;
        EtherCAT_Motor[joint_ids[i]].control.Position = 0;
        EtherCAT_Motor[joint_ids[i]].control.Speed = 0.5;
        EtherCAT_Motor[joint_ids[i]].control.Torque = 3;
    }

    while (ros::ok()) {
        // 创建并填充 JointState 消息
        sensor_msgs::JointState joint_state;
        auto time_now = ros::Time::now();
        joint_state.header.stamp = time_now;

        for (size_t i = 0; i < 16; ++i) {
            joint_state.name.push_back(joint_names[i]);
            joint_state.position.push_back(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Position_Actual);
            joint_state.velocity.push_back(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Speed_Actual);
            joint_state.effort.push_back(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Torque_Actual);
        }

        // 发布消息
        joint_state_pub.publish(joint_state);

        // 设置左侧和右侧夹持器的位置
        EtherCAT_Motor[joint_ids[7]].control.Position = joint_pos_minus[7] * left_gripper_control;
        EtherCAT_Motor[joint_ids[15]].control.Position = joint_pos_minus[15] * right_gripper_control;

        // 设置左侧关节的位置
        for (int i = 0; i < 7; ++i) {
            EtherCAT_Motor[joint_ids[i]].control.Position = joint_pos_minus[i] * left_joint_control[i];
        }

        // 设置右侧关节的位置
        for (int i = 8; i < 15; ++i) {
            EtherCAT_Motor[joint_ids[i]].control.Position = joint_pos_minus[i] * right_joint_control[i];
        }

        // 按照设定的频率休眠
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
