#include <vector>

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Float32MultiArray.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class RobotArm
{
  private:
    // Action client for the joint trajectory action
    // used to trigger the arm movement action
    TrajClient *traj_client_;

  public:
    //! Initialize the action client and wait for action server to come up
    RobotArm()
    {
        // tell the action client that we want to spin a thread by default
        traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

        // wait for action server to come up
        while (!traj_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }
    }

    //! Clean up the action client
    ~RobotArm()
    {
        delete traj_client_;
    }

    //! Sends the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
    {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        traj_client_->sendGoal(goal);
    }

    //! Generates a simple trajectory with two waypoints, used as an example
    /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
    pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory(std_msgs::Float32MultiArray points)
    {
        //our goal variable
        pr2_controllers_msgs::JointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(points.layout.dim[0].size);

        for (size_t ind = 0; ind < points.layout.dim[0].size; ++ind)
        {
            for (size_t i = 0; i < points.layout.dim[1].size; ++i)
            {
                goal.trajectory.points[ind].positions.resize(points.layout.dim[1].size);
                goal.trajectory.points[ind].positions[i] = points.data[ind * points.layout.dim[1].stride + i];

                // Velocities
                goal.trajectory.points[ind].velocities.resize(points.layout.dim[1].size);
                goal.trajectory.points[ind].velocities[i] = 0.0;

                // To be reached 1 second after starting along the trajectory
                goal.trajectory.points[ind].time_from_start = ros::Duration(ind + 1.0);
            }
        }

        //we are done; return the goal
        return goal;
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
        return traj_client_->getState();
    }

};

void print_joint_values(robot_model::RobotModelPtr kinematic_model, robot_state::RobotStatePtr kinematic_state)
{
    // kinematic_state->setToDefaultValues();

    std::cout << "Joint Model Groups " << kinematic_model->getJointModelGroupNames().size() << std::endl;
    for (int i=0; i<kinematic_model->getJointModelGroupNames().size(); i++)
        std::cout << kinematic_model->getJointModelGroupNames()[i] << std::endl;


    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    std::cout << "Joint Model Names: " << kinematic_model->getJointModelNames().size() << std::endl;

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for(std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }


    // std::vector<std::string> joint_names;
    // for (std::size_t i = 0; i < kinematic_model->getJointModelNames().size(); ++i)
    // {
    //     // std::string joint_name = kinematic_model->getJointModelNames()[i];
    //     // joint_names.push_back(joint_name);
    //     // const double *pos = kinematic_state->getJointPositions(joint_name);
    //     // joint_values.push_back(*pos);
    //     ROS_INFO("Joint %d %s: %f", i, joint_names[i].c_str(), joint_values[i]);
    // }

    // const double *pos = kinematic_state->getJointPositions("r_shoulder_pan_joint");
    // ROS_INFO("Joint r_shoulder_pan_joint: %f", *pos);

    // /* Set one joint in the right arm outside its joint limit */
    // joint_values[0] = 0.0;
    // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    // /* Check whether any joint is outside its joint limits */
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // /* Enforce the joint limits for this state and check again*/
    // kinematic_state->enforceBounds();
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());


    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                             reference_point_position,
                             jacobian);
    ROS_INFO_STREAM("Jacobian: " << jacobian);
}
    
void trajectoryCallback(const std_msgs::Float32MultiArray &traj)
{
    std::cout << traj << std::endl;
    RobotArm arm;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    // arm.print_joint_values();

    arm.startTrajectory(arm.armExtensionTrajectory(traj));

    // Wait for trajectory completion
    while (!arm.getState().isDone() && ros::ok())
    {
        usleep(50000);
    }
    print_joint_values(kinematic_model, kinematic_state);
}

int main(int argc, char **argv)
{
    // Init the ROS node
    ros::init(argc, argv, "robot_driver");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("trajectory", 1, trajectoryCallback);

    // for (int i = 0; i < kinematic_model->getJointModelNames().size(); ++i)
    // {
    //     std::cout << i << " " << kinematic_model->getJointModelNames()[i] << std::endl;
    // }

    std::cout << "Running trajectory controller" << std::endl;
    ros::spin();
    return 0;
}