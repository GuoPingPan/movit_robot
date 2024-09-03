#include "EtherCAT_Motor.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char** argv){

    ros::init(argc, argv, "motion_planning");

    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);


    ros::Rate loop_rate(200);

    int traject_count = 0;

    std::string joint_names[16] = {
        "left_1_joint", "left_2_joint", "left_3_joint", "left_4_joint",
        "left_5_joint", "left_6_joint", "left_7_joint", "left_8_joint",
        "right_1_joint", "right_2_joint", "right_3_joint", "right_4_joint",
        "right_5_joint", "right_6_joint", "right_7_joint", "right_8_joint"
    };

    float joint_pos_minus[16] = {
        -1, -1, -1, -1,
        1, 1, 1, -1,
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
    if (ret == FALSE){
        return 1;
    }

    //TODO 这里要初始化每个电机的控制方式, 位置等

    for(int i = 0; i <  sizeof(joint_names) / sizeof(joint_names[0]); ++i){
        EtherCAT_Motor[joint_ids[i]].control.Control_Mode = MIX_CONTROL;
        EtherCAT_Motor[joint_ids[i]].control.KP = 0;
        EtherCAT_Motor[joint_ids[i]].control.KD = 0;
        EtherCAT_Motor[joint_ids[i]].control.Position = 0;
        EtherCAT_Motor[joint_ids[i]].control.Speed = 0;
        EtherCAT_Motor[joint_ids[i]].control.Torque = 0;
    }

    while (ros::ok())
    {
        // 创建并填充 JointState 消息
        sensor_msgs::JointState joint_state;
        auto time_now = ros::Time::now();
        joint_state.header.stamp = time_now;

        for (size_t i = 0; i < 16; ++i)
        {
            joint_state.name.push_back(joint_names[i]);
            // 这里需要替换 Motor_Control 和相关字段的实际实现
            joint_state.position.push_back(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Position_Actual);
            joint_state.velocity.push_back(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Speed_Actual);
            joint_state.effort.push_back(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Torque_Actual);
        }

        // 发布消息
        joint_state_pub.publish(joint_state);

        // 按照设定的频率休眠
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;

}
