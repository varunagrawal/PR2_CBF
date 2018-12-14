Simple ROS node to publish a fixed trajectory for the PR2.

Consider this a baseline.

We first need to run `rosrun joint_states_listener joint_states_listener.py` to be able to read joint states of the robot, and then we can run `python src/trajectory_publisher.py` to publish the trajectory.