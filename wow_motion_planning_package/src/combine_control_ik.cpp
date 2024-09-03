#include <iostream>                      // 用于标准输入输出
#include <vector>                       // 用于 std::vector
#include "pinocchio/fwd.hpp"
#include <ros/ros.h>                    // ROS 核心库
#include <std_msgs/Float32MultiArray.h> // ROS 消息类型
#include <sensor_msgs/JointState.h>     // ROS 关节状态消息类型
#include "pinocchio/multibody/sample-models.hpp" // Pinocchio 机器人模型
#include "pinocchio/spatial/explog.hpp" // Pinocchio 空间数学
#include "pinocchio/algorithm/kinematics.hpp" // Pinocchio 运动学
#include "pinocchio/algorithm/jacobian.hpp"   // Pinocchio 雅可比矩阵
#include "pinocchio/algorithm/joint-configuration.hpp" // Pinocchio 关节配置
#include "pinocchio/parsers/urdf.hpp"  // Pinocchio URDF 解析
#include <Eigen/Core>                  // Eigen 核心功能
#include <Eigen/Geometry>              // Eigen 几何变换
#include <filesystem>

// Global variables
Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(16);
Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(16);
Eigen::Vector3d left_target_position = Eigen::Vector3d::Zero(3);
Eigen::Vector3d left_target_rpy = Eigen::Vector3d::Zero(3);
Eigen::Vector3d right_target_position = Eigen::Vector3d::Zero(3);
Eigen::Vector3d right_target_rpy = Eigen::Vector3d::Zero(3);


// Joint state callback
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    joint_positions = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
    joint_velocities = Eigen::VectorXd::Map(msg->velocity.data(), msg->velocity.size());
}

void LeftTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // 确保消息数据的大小正确
    if (msg->data.size() != 6) {
        ROS_WARN("Received target message with unexpected size: %zu", msg->data.size());
        return;
    }

    // 提取目标位置和姿态
    left_target_position = Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
    left_target_rpy = Eigen::Vector3d(msg->data[3], msg->data[4], msg->data[5]);

    // 你可以在这里添加更多的处理逻辑
    ROS_INFO("Target position updated: [%f, %f, %f]", left_target_position[0], left_target_position[1], left_target_position[2]);
    ROS_INFO("Target RPY updated: [%f, %f, %f]", left_target_rpy[0], left_target_rpy[1], left_target_rpy[2]);
}

void rightTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    // 确保消息数据的大小正确
    if (msg->data.size() != 6) {
        ROS_WARN("Received target message with unexpected size: %zu", msg->data.size());
        return;
    }

    // 提取目标位置和姿态
    right_target_position = Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
    right_target_rpy = Eigen::Vector3d(msg->data[3], msg->data[4], msg->data[5]);

    // 你可以在这里添加更多的处理逻辑
    ROS_INFO("Target position updated: [%f, %f, %f]", right_target_position[0], right_target_position[1], right_target_position[2]);
    ROS_INFO("Target RPY updated: [%f, %f, %f]", right_target_rpy[0], right_target_rpy[1], right_target_rpy[2]);
}


// Convert RPY to Rotation Matrix
Eigen::Matrix3d rpy_to_rotation_matrix(const Eigen::Vector3d& rpy) {
    Eigen::AngleAxisd rollAngle(rpy[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy[2], Eigen::Vector3d::UnitZ());
    
    Eigen::Matrix3d rotationMatrix = yawAngle.toRotationMatrix() * pitchAngle.toRotationMatrix() * rollAngle.toRotationMatrix();
    return rotationMatrix;
}


bool fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();  // 返回 true 如果文件存在且可以打开
}


int main(int argc, char **argv) {
    using namespace pinocchio;

    ros::init(argc, argv, "pinocchio_example_node");
    ros::NodeHandle nh;

    ros::Subscriber joint_state_sub = nh.subscribe("joint_state", 1, JointStateCallback);
    ros::Subscriber left_target_sub = nh.subscribe("left_target", 1, LeftTargetCallback);
    ros::Publisher left_joint_control_pub = nh.advertise<std_msgs::Float32MultiArray>("left_joint_control", 1);
    ros::Subscriber right_target_sub = nh.subscribe("right_target", 1, rightTargetCallback);
    ros::Publisher right_joint_control_pub = nh.advertise<std_msgs::Float32MultiArray>("right_joint_control", 1);

    // std::filesystem::path exePath = std::filesystem::canonical("/proc/self/exe");
    // std::cout << "Current working directory: " << exePath << std::endl;

    // Load the URDF model
    // const std::string urdf_filename = exePath.string() + "/../../wow_description/up_body.urdf";
    const std::string urdf_filename = "/home/jetson/wow_ws/src/wow_description/up_body.urdf";
    if (fileExists(urdf_filename)) {
        std::cout << "File exists: " << urdf_filename << std::endl;
    } else {
        std::cout << "File does not exist: " << urdf_filename << std::endl;
    }
    
    Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    
    pinocchio::Data data(model);
    const double eps = 2e-3;
    const int IT_MAX = 5000;
    const double DT = 1e-1;
    const double damp = 1e-12;
    const int left_JOINT_ID = 7;  // 8 
    const int right_JOINT_ID = 15;  // 8 

    Eigen::VectorXd q = pinocchio::neutral(model);
    q.setZero();

    pinocchio::Data::Matrix6x J(6, model.nv);
    J.setZero();

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::SE3 temp = data.oMi[left_JOINT_ID];
    Eigen::Vector3d left_origin = temp.translation();
    pinocchio::SE3 temp1 = data.oMi[right_JOINT_ID];
    Eigen::Vector3d right_origin = temp1.translation();

    std::cout <<"left origin: " << left_origin << std::endl; 
    std::cout <<"right origin: " << right_origin << std::endl; 

    // Main loop
    while (ros::ok()) {
        // Update joint configuration from joint state
        q = joint_positions;
        std::cout << "joint_positions: " << joint_positions.transpose() << std::endl;
        std::cout << "q: " << q.transpose() << std::endl;

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::SE3 now = data.oMi[left_JOINT_ID];

        // Target position and orientation
        const pinocchio::SE3 oMdes(rpy_to_rotation_matrix(left_target_rpy), left_target_position+left_origin);

        std::vector<pinocchio::SE3> interp_seq;
        for (int i = 1; i < 2; ++i) {
            float alpha = 1 * i;
            pinocchio::SE3 temp = pinocchio::SE3::Interpolate(now, oMdes, alpha);
            interp_seq.push_back(temp);
        }

        typedef Eigen::Matrix<double, 6, 1> Vector6d;
        Vector6d err;
        Eigen::VectorXd v(model.nv);

        for (const auto& target : interp_seq) {
            std::cout << "Target: " << target << std::endl;
            bool success = false;
            int i = 0;
            for (; i < IT_MAX; ++i) {

                pinocchio::forwardKinematics(model, data, q);
                const pinocchio::SE3 iMd = data.oMi[left_JOINT_ID].actInv(target);
                err = pinocchio::log6(iMd).toVector(); // in joint frame

                if (err.norm() < eps) {
                    success = true;
                    break;
                }

                J.setZero();
                pinocchio::computeJointJacobian(model, data, q, left_JOINT_ID, J); // J in joint frame
            
                // std::cout << "---------------- left J -----------------" << std::endl;
                // std::cout << J << std::endl;
                // std::cout << "---------------- left J -----------------" << std::endl;

                pinocchio::Data::Matrix6 Jlog;
                pinocchio::Jlog6(iMd.inverse(), Jlog);
                J = -Jlog * J;

              

                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;
                v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
                q = pinocchio::integrate(model, q, v * DT);

                // Clip q to ensure it's within bounds
                q = q.cwiseMin(model.upperPositionLimit).cwiseMax(model.lowerPositionLimit);
            }

            if (success) {
                std::cout << "Convergence achieved!" << std::endl;
                std_msgs::Float32MultiArray joint_control_msg;
                joint_control_msg.data = std::vector<float>(q.data(), q.data() + q.size());
                left_joint_control_pub.publish(joint_control_msg);
            } else {
                std::cout << "Warning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
            }

            std::cout << "left iter: " << i << std::endl;
            std::cout << "left Result: " << q.transpose() << std::endl;
            std::cout << "left Final error: " << err.transpose() << std::endl;
        }


        // TODO 这里是否需要考虑更新？
        // q = joint_positions;
        std::cout << "joint_positions: " << joint_positions.transpose() << std::endl;
        std::cout << "q: " << q.transpose() << std::endl;

        pinocchio::forwardKinematics(model, data, q);
        pinocchio::SE3 now1 = data.oMi[right_JOINT_ID];

        // Target position and orientation
        const pinocchio::SE3 oMdes1(rpy_to_rotation_matrix(right_target_rpy), right_target_position+right_origin);

        std::vector<pinocchio::SE3> interp_seq1;
        for (int i = 1; i < 2; ++i) {
            float alpha = 1 * i;
            pinocchio::SE3 temp = pinocchio::SE3::Interpolate(now1, oMdes1, alpha);
            interp_seq1.push_back(temp);
        }

        Vector6d err1;
        Eigen::VectorXd v1(model.nv);

        for (const auto& target : interp_seq1) {
            std::cout << "Target: " << target << std::endl;
            bool success = false;
            int i = 0;
            for (; i < IT_MAX; ++i) {

                pinocchio::forwardKinematics(model, data, q);
                const pinocchio::SE3 iMd = data.oMi[right_JOINT_ID].actInv(target);
                err1 = pinocchio::log6(iMd).toVector(); // in joint frame

                if (err1.norm() < eps) {
                    success = true;
                    break;
                }

                J.setZero();
                pinocchio::computeJointJacobian(model, data, q, right_JOINT_ID, J); // J in joint frame
                // std::cout << "---------------- left R -----------------" << std::endl;
                // std::cout << J << std::endl;
                // std::cout << "---------------- left R -----------------" << std::endl;

                pinocchio::Data::Matrix6 Jlog;
                pinocchio::Jlog6(iMd.inverse(), Jlog);
                J = -Jlog * J;
                
                pinocchio::Data::Matrix6 JJt;
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += damp;
                v1.noalias() = -J.transpose() * JJt.ldlt().solve(err1);
                q = pinocchio::integrate(model, q, v1 * DT);

                // Clip q to ensure it's within bounds
                q = q.cwiseMin(model.upperPositionLimit).cwiseMax(model.lowerPositionLimit);
            }

            if (success) {
                std::cout << "Convergence achieved!" << std::endl;
                std_msgs::Float32MultiArray joint_control_msg;
                joint_control_msg.data = std::vector<float>(q.data(), q.data() + q.size());
                right_joint_control_pub.publish(joint_control_msg);
            } else {
                std::cout << "Warning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
            }

            std::cout << "right iter: " << i << std::endl;
            std::cout << "right Result: " << q.transpose() << std::endl;
            std::cout << "right Final error: " << err.transpose() << std::endl;
        }



        ros::spinOnce();
    }

    return 0;
}
