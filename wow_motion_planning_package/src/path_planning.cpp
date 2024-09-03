#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <iostream>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_tutorial");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    int count = 1;
    std::cout << count << std::endl;
    count += 1;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    /* listen for planning scene messages on topic /XXX and apply them to
                         the internal planning scene accordingly */
    psm->startSceneMonitor();
    /* listens to changes of world geometry, collision objects, and (optionally) octomaps */
    psm->startWorldGeometryMonitor();
    /* listen to joint state updates as well as changes in attached collision objects
                          and update the internal planning scene accordingly*/
    
    std::cout << count << std::endl;
    count += 1;
    psm->startStateMonitor();
    std::cout << count << std::endl;
    count += 1;
    
    /* We can also use the RobotModelLoader to get a robot model which contains the robot's kinematic information */
    moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

    /* We can get the most up to date robot state from the PlanningSceneMonitor by locking the internal planning scene
       for reading. This lock ensures that the underlying scene isn't updated while we are reading it's state.
       RobotState's are useful for computing the forward and inverse kinematics of the robot among many other uses */
    moveit::core::RobotStatePtr robot_state(
        new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

    std::cout << count << std::endl;
    count += 1;

    /* Create a JointModelGroup to keep track of the current robot pose and planning group. The Joint Model
       group is useful for dealing with one set of joints at a time such as a left arm or a end effector */
    const moveit::core::JointModelGroup *left_joint_model_group = robot_state->getJointModelGroup("left_arm");
    const moveit::core::JointModelGroup *right_joint_model_group = robot_state->getJointModelGroup("right_arm");


    // We can now setup the PlanningPipeline object, which will use the ROS parameter server
    // to determine the set of request adapters and the planning plugin to use
    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the right arm of the Panda
    // specifying the desired pose of the end-effector as input.
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    req.group_name = "left_arm";
    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("left_7_link", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    // Before planning, we will need a Read Only lock on the planning scene so that it does not modify the world
    // representation while planning
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }

    /* Now, call the pipeline and check whether planning was successful. */
    /* Check that the planning was successful */
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    std::cout << count << std::endl;
    count += 1;

    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    ROS_INFO("Visualizing the trajectory");
    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), left_joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    std::cout << count << std::endl;
    count += 1;

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
    robot_state->setJointGroupPositions(left_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
    moveit::core::RobotState goal_state(*robot_state);
    std::vector<double> joint_values = {-1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0};
    goal_state.setJointGroupPositions(left_joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, left_joint_model_group);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }

    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), left_joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // Using a Planning Request Adapter
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // A planning request adapter allows us to specify a series of operations that
    // should happen either before planning takes place or after the planning
    // has been done on the resultant path

    /* First, set the state in the planning scene to the final state of the last plan */
    robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
    robot_state->setJointGroupPositions(left_joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

    // Now, set one of the joints slightly outside its upper limit
    const moveit::core::JointModel *joint_model = left_joint_model_group->getJointModel("left_3_joint");
    const moveit::core::JointModel::Bounds &joint_bounds = joint_model->getVariableBounds();
    std::vector<double> tmp_values(1, 0.0);
    tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
    robot_state->setJointPositions(joint_model, tmp_values);

    req.goal_constraints.clear();
    req.goal_constraints.push_back(pose_goal);

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }
    /* Visualize the trajectory */
    ROS_INFO("Visualizing the trajectory");
    res.getMessage(response);
    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    /* Now you should see three planned trajectories in series*/
    display_publisher.publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), left_joint_model_group);
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");

    ROS_INFO("Done");
    return 0;
}
