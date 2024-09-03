#include "EtherCAT_Motor.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <sensor_msgs/JointState.h>



moveit_msgs::DisplayTrajectory trajectory;

// Callback 函数处理接收到的 DisplayTrajectory 消息
void displayTrajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
{
    ROS_INFO("Received DisplayTrajectory message");

    // 打印出接收到的轨迹数量
    ROS_INFO("Number of trajectories: %lu", msg->trajectory.size());
    
    for (size_t i = 0; i < msg->trajectory.size(); ++i)
    {
        ROS_INFO("Trajectory %zu", i);
        ROS_INFO("  Number of points: %zu", msg->trajectory[i].joint_trajectory.points.size());
    }

    // 更新全局变量
    trajectory = *msg;
}



int main(int argc, char** argv){

    ros::init(argc, argv, "get_trajectory");


    ros::NodeHandle nh;

    // 创建一个订阅者，订阅 "/move_group/display_planned_path" 话题
    ros::Subscriber sub = nh.subscribe("/move_group/display_planned_path", 10, displayTrajectoryCallback);
    ros::Rate loop_rate(200);


    //TODO 这里要初始化每个电机的控制方式, 位置等


    while (ros::ok())
    {
 
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;

}
