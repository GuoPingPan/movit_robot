#include "EtherCAT_Motor.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>


moveit_msgs::DisplayTrajectory right_trajectory;


float right_open = 0;

// 回调函数
void gripperRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // 处理接收到的消息
    // ROS_INFO("Received right gripper value: %f", msg->data);
    right_open = msg->data;
}

float left_open = 0;

// 回调函数
void gripperLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // 处理接收到的消息
    // ROS_INFO("Received left gripper value: %f", msg->data);
    left_open = msg->data;
}

int left_trajectory_receive = 0;
int right_trajectory_receive = 0;


// Callback 函数处理接收到的 DisplayTrajectory 消息
void RightdisplayTrajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
    // 打印出接收到的轨迹数量
    ROS_INFO("Number of trajectories: %lu", msg->trajectory.size());

    if (msg->trajectory.size() > 0)
    {
        right_trajectory_receive += 1;
    }

    for (size_t i = 0; i < msg->trajectory.size(); ++i)
    {
        ROS_INFO("[RIGHT] Trajectory %zu", i);
        ROS_INFO("  Number of points: %zu", msg->trajectory[i].joint_trajectory.points.size());
    }

    // 更新全局变量
    right_trajectory = *msg;

    if ((!right_trajectory.trajectory.empty()) && (!right_trajectory.trajectory[0].joint_trajectory.points.empty()))
    {
        right_trajectory.trajectory[0].joint_trajectory.points.erase(right_trajectory.trajectory[0].joint_trajectory.points.begin());
    }
}

moveit_msgs::DisplayTrajectory left_trajectory;

// Callback 函数处理接收到的 DisplayTrajectory 消息
void LeftdisplayTrajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
    // 打印出接收到的轨迹数量
    ROS_INFO("Number of trajectories: %lu", msg->trajectory.size());

    if (msg->trajectory.size() > 0)
    {
        left_trajectory_receive += 1;
    }

    for (size_t i = 0; i < msg->trajectory.size(); ++i)
    {
        ROS_INFO("[LEFT] Trajectory %zu", i);
        ROS_INFO("  Number of points: %zu", msg->trajectory[i].joint_trajectory.points.size());
    }

    // 更新全局变量
    left_trajectory = *msg;

    if ((!left_trajectory.trajectory.empty()) && (!left_trajectory.trajectory[0].joint_trajectory.points.empty()))
    {
        left_trajectory.trajectory[0].joint_trajectory.points.erase(left_trajectory.trajectory[0].joint_trajectory.points.begin());
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "motion_planning");

    ros::NodeHandle nh;
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // 创建一个订阅者，订阅 "/move_group/display_planned_path" 话题
    ros::Subscriber left_sub = nh.subscribe("/move_group/left_display_planned_path", 1, LeftdisplayTrajectoryCallback);
    ros::Subscriber right_sub = nh.subscribe("/move_group/right_display_planned_path", 1, RightdisplayTrajectoryCallback);
    ros::Subscriber left_gripper_sub = nh.subscribe("/gripper_left", 1, gripperLeftCallback);
    ros::Subscriber right_gripper_sub = nh.subscribe("/gripper_right", 1, gripperRightCallback);
    ros::Rate loop_rate(500);

    int traject_count = 0;
    float last_right_open = right_open;
    float last_left_open = left_open;
    int left_traject_receive = left_trajectory_receive;
    int right_traject_receive = right_trajectory_receive;

    std::string joint_names[16] = {
        "left_1_joint", "left_2_joint", "left_3_joint", "left_4_joint",
        "left_5_joint", "left_6_joint", "left_7_joint", "left_8_joint",
        "right_1_joint", "right_2_joint", "right_3_joint", "right_4_joint",
        "right_5_joint", "right_6_joint", "right_7_joint", "right_8_joint"
    };

    // 这个和关节名称对应
    float joint_pos_minus[16] = {
        -1, -1, -1, -1,
        1, 1, 1, 1,
        -1, 1, -1, -1,
        1, 1, 1, -1,
    };

    // 这已经把关节电机的值调成了 joint_names 的顺序
    int joint_ids[16] = {
        7, 6, 5, 4,
        3, 2, 1, 0,
        8, 9, 10, 11,
        12, 13, 14, 15
    };


    int ret = EtherCAT_Init(DC_Sync, 5000);

    if(ret == FALSE)
        return 0;

    // TODO 这里要初始化每个电机的控制方式, 位置等

    for (int i = 0; i < sizeof(joint_names) / sizeof(joint_names[0]); ++i)
    {
        EtherCAT_Motor[joint_ids[i]].control.Control_Mode = POS_CONTROL;
        EtherCAT_Motor[joint_ids[i]].control.KP = 0;
        EtherCAT_Motor[joint_ids[i]].control.KD = 0;
        EtherCAT_Motor[joint_ids[i]].control.Position = 0;
        EtherCAT_Motor[joint_ids[i]].control.Speed = 3;
        EtherCAT_Motor[joint_ids[i]].control.Torque = 3;
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
        // ROS_INFO("UPDATE JOINT STATS.");
        joint_state_pub.publish(joint_state);

        // TODO: 执行轨迹
        if ((!left_trajectory.trajectory.empty()) 
                && (!left_trajectory.trajectory[0].joint_trajectory.points.empty())
            )
        {
            // element 的顺序就是 joint_names 的顺序
            auto &first_element =left_trajectory.trajectory[0].joint_trajectory.points.back();
            // auto &first_element = left_trajectory.trajectory[0].joint_trajectory.points.front();

            // first_element.velocities
            // first_element.accelerations
            // first_element.effort

            float tolerant = 0.1;

            for (int i = 0; i < 7; ++i)
            {

                EtherCAT_Motor[joint_ids[i]].control.Position = joint_pos_minus[i] * first_element.positions[i];
                // ROS_INFO("[LEFT] JOINT TARGET %s: %f rad", joint_names[i].c_str(), EtherCAT_Motor[joint_ids[i]].control.Position);
                // ROS_INFO("[LEFT] JOINT ACTUAL %s: %f rad", joint_names[i].c_str(), EtherCAT_Motor[joint_ids[i]].information.Position_Actual);
            }

            bool within_tol = true;

            for (int j = 0; j < 7; ++j)
            {
                float delta = std::abs(EtherCAT_Motor[joint_ids[j]].information.Position_Actual - joint_pos_minus[j] * first_element.positions[j]);
                // ROS_INFO("[LEFT] DELTA OF JOINT %s: %f", joint_names[j].c_str(), delta);
                if (delta > tolerant)
                {
                    within_tol = false;
                }

            
            }
            if (within_tol)
            {
                ROS_INFO("[LEFT] SUCCESS ARRIVE TARGET!");
                left_trajectory.trajectory[0].joint_trajectory.points.erase(left_trajectory.trajectory[0].joint_trajectory.points.begin());
            }

            // ROS_INFO("trajectory len: %ld", left_trajectory.trajectory[0].joint_trajectory.points.size());
        }
        else
        {
            // ROS_INFO("No trajectory to execute");

            // keep current position
        }

        if (left_open != last_left_open){
            ROS_INFO("OPEN LEFT GRIPPER");
            EtherCAT_Motor[joint_ids[7]].control.Position = joint_pos_minus[7] * left_open;
            last_left_open = left_open;
        }


        // TODO: 执行轨迹
        if ((!right_trajectory.trajectory.empty()) 
                && (!right_trajectory.trajectory[0].joint_trajectory.points.empty())
            )
        {
            // element 的顺序就是 joint_names 的顺序
            auto &first_element = right_trajectory.trajectory[0].joint_trajectory.points.back();
            // auto &first_element = right_trajectory.trajectory[0].joint_trajectory.points.front();

            // first_element.velocities
            // first_element.accelerations
            // first_element.effort

            float tolerant = 0.1;

            for (int i = 8; i < 15; ++i)
            {

                EtherCAT_Motor[joint_ids[i]].control.Position = joint_pos_minus[i] * first_element.positions[i-8];
                // ROS_INFO("[RIGHT] JOINT TARGET %s: %f rad", joint_names[i].c_str(), EtherCAT_Motor[joint_ids[i]].control.Position);
                // ROS_INFO("[RIGHT] JOINT ACTUAL %s: %f rad", joint_names[i].c_str(), EtherCAT_Motor[joint_ids[i]].information.Position_Actual);
            }

            bool within_tol = true;

            for (int j = 8; j < 15; ++j)
            {
                float delta = std::abs(EtherCAT_Motor[joint_ids[j]].information.Position_Actual - joint_pos_minus[j] * first_element.positions[j-8]);
                // ROS_INFO("[RIGHT] DELTA OF JOINT %s: %f", joint_names[j].c_str(), delta);
                if (delta > tolerant)
                {
                    within_tol = false;
                }

            
            }
            if (within_tol)
            {
                ROS_INFO("[RIGHT] SUCCESS ARRIVE TARGET!");
                right_trajectory.trajectory[0].joint_trajectory.points.erase(right_trajectory.trajectory[0].joint_trajectory.points.begin());
            }

            // ROS_INFO("trajectory len: %ld", right_trajectory.trajectory[0].joint_trajectory.points.size());
        }
        else
        {
            // ROS_INFO("No trajectory to execute");

            // keep current position
        }

        if (right_open != last_right_open){
            ROS_INFO("OPEN RIGHT GRIPPER");
            EtherCAT_Motor[joint_ids[15]].control.Position = joint_pos_minus[15] * right_open;
            last_right_open = right_open;
        }



        // if ((!right_trajectory.trajectory.empty())
        //         && (!right_trajectory.trajectory[0].joint_trajectory.points.empty())
        //     )
        // {
        //     auto &first_element = right_trajectory.trajectory[0].joint_trajectory.points.front();

        //     // first_element.velocities
        //     // first_element.accelerations
        //     // first_element.effort

        //     ROS_INFO("size: %d", first_element.positions.size());

        //     for (int i = 8; i < 15; ++i)
        //     {
        //         // ROS_INFO("[RIGHT] JOINT %d: %f", i, joint_pos_minus[i] * first_element.positions[i-8]);
        //         EtherCAT_Motor[joint_ids[i]].control.Position = joint_pos_minus[i] * first_element.positions[i-8];
        //     }
        //     float tolerant = 0.02;

        //     while (true)
        //     {
        //         bool within_tol = true;

        //         for (int i = 8; i < 15; ++i)
        //         {   
        //             float delta = std::abs(joint_pos_minus[i] * EtherCAT_Motor[joint_ids[i]].information.Position_Actual - first_element.positions[i-8]);
        //             // ROS_INFO("[RIGHT] DELTA OF JOINT %s: %f", joint_names[i].c_str(), delta);
        //             if (delta > tolerant)
        //             {
        //                 within_tol = false;
        //             }
        //         }
        //         if (within_tol)
        //         {
        //             break;
        //         }

        //         usleep(2000);
        //     }
        //     ROS_INFO("[RIGHT] SUCCESS ARRIVE TARGET!");


        //     // 用完后丢弃
        //     right_trajectory.trajectory[0].joint_trajectory.points.erase(right_trajectory.trajectory[0].joint_trajectory.points.begin());
        // }
        // else
        // {
        //     // ROS_INFO("No trajectory to execute");

        //     // keep current position
        // }

        // if (right_open != last_right_open){
        //     EtherCAT_Motor[joint_ids[15]].control.Position = joint_pos_minus[15] * right_open;
        //     last_right_open = right_open;
        // }

        // 按照设定的频率休眠
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
