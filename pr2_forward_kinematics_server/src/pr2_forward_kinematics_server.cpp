#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32MultiArray.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

bool callback(moveit_msgs::GetPositionFKRequest &req, 
              moveit_msgs::GetPositionFKResponse &res)
{
  ROS_INFO("request: ", req);
  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PR2_Forward_Kinematics_Server");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<moveit_msgs::GetPositionFK>("pr2_forward_kinematics");
    moveit_msgs::GetPositionFK srv;
    ROS_INFO(srv);
    // srv.request.a = atoll(argv[1]);
    // srv.request.b = atoll(argv[2]);
    // if (client.call(srv))
    // {
    //     ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service add_two_ints");
    //     return 1;
    // }

    return 0;
}