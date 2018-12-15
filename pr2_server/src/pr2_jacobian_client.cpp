#include "ros/ros.h"
#include <pr2_server/Jacobian.h>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pr2_fk_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pr2_server::Jacobian>("get_pr2_jacobian");

    pr2_server::Jacobian srv;

    srv.request.joint_state.name = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
    srv.request.joint_state.position = {-0.787064843322, -4.22593924299e-07, -1.75779266698e-07, -0.149996749925, 2.28851725126e-07, -0.100002260175, 1.70990356096e-07};
    srv.request.joint_state.velocity.resize(7, 0.0);
    srv.request.joint_state.effort.resize(7, 0.0);

    // std::cout << srv.request << std::endl;

    if (client.call(srv))
    {
        std::cout << srv.response << std::endl;
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}

/*
joint names:	r_shoulder_pan_joint	r_shoulder_lift_joint	r_upper_arm_roll_joint	r_elbow_flex_joint	r_forearm_roll_joint	r_wrist_flex_joint	r_wrist_roll_joint
position:	-0.787064843322	-4.22593924299e-07	-1.75779266698e-07	-0.149996749925	2.28851725126e-07	-0.100002260175	1.70990356096e-07
velocity:	2.76377066572e-07	0.000151913413794	4.42465340408e-05	-0.000196080093519	-4.10143790073e-05	4.2948874999e-05	1.51794756889e-06
effort:	-1.41527719062e-05	0.00312602274897	5.23244604675e-05	-0.000386927456318	-2.65101312718e-05	9.16861348759e-06	-0.000106370019846

*/