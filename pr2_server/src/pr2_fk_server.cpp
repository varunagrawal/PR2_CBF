#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32MultiArray.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/GetPositionFK.h>

#include <iostream>
#include <vector>

bool callback(moveit_msgs::GetPositionFK::Request &req,
              moveit_msgs::GetPositionFK::Response &res)
{
    // Setup the robot state
    ROS_INFO("request: ", req);
    robot_model_loader::RobotModelLoader robot_loader = robot_model_loader::RobotModelLoader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_loader.getModel();

    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    planning_scene::PlanningScene ps(kinematic_model);
    ps.getCurrentStateNonConst().update();

    // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    robot_state::RobotState &kinematic_state = ps.getCurrentStateNonConst();

    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    // Initialize the joint values
    std::vector<double> joint_values;

    kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);

    // copy the joint values from the request
    for (int i = 0; i < joint_names.size(); ++i)
    {
        for (int j = 0; j < req.robot_state.joint_state.name.size(); ++j)
        {
            if (joint_names[i] == req.robot_state.joint_state.name[j])
            {
                joint_values[i] = req.robot_state.joint_state.position[j];
            }
        }
    }

    kinematic_state.setJointGroupPositions(joint_model_group, joint_values);

    // kinematic_state.setVariablePositions(req.robot_state.joint_state.position);
    // kinematic_state.update();

    /* Enforce the joint limits for this state and check joint limits*/
    // kinematic_state->enforceBounds();
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    const Eigen::Affine3d &end_effector_state = kinematic_state.getGlobalLinkTransform("r_wrist_roll_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_DEBUG_STREAM("Translation: " << end_effector_state.translation());
    ROS_DEBUG_STREAM("Rotation: " << end_effector_state.rotation());

    geometry_msgs::Pose p;
    tf::poseEigenToMsg(end_effector_state, p);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = p;
    res.pose_stamped.push_back(pose_stamped);
    // std::cout << res << std::endl;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PR2_Forward_Kinematics_Server");

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("get_pr2_forward_kinematics", callback);
    ROS_INFO("Ready!");
    ros::spin();

    return 0;
}