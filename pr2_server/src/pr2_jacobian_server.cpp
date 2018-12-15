#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float64MultiArray.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <pr2_server/Jacobian.h>

#include <iostream>
#include <vector>

bool callback(pr2_server::Jacobian::Request &req,
              pr2_server::Jacobian::Response &res)
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
        for (int j = 0; j < req.joint_state.name.size(); ++j)
        {
            if (joint_names[i] == req.joint_state.name[j])
            {
                joint_values[i] = req.joint_state.position[j];
            }
        }
    }

    kinematic_state.setJointGroupPositions(joint_model_group, joint_values);

    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state.getJacobian(joint_model_group, kinematic_state.getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position,
                                jacobian);
    ROS_INFO_STREAM("Jacobian: " << jacobian);

    tf::matrixEigenToMsg(jacobian, res.jacobian);

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PR2_Jacobian_Server");

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("get_pr2_jacobian", callback);
    ROS_INFO("Ready!");
    ros::spin();

    return 0;
}