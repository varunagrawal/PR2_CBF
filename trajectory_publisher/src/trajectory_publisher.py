import time
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import roslib

from joint_state_reader import get_joint_states

def get_joint_values():
    joint_names = ["r_shoulder_pan_joint",
                   "r_shoulder_lift_joint",
                    "r_upper_arm_roll_joint",
                    "r_elbow_flex_joint",
                    "r_forearm_roll_joint",
                    "r_wrist_flex_joint",
                    "r_wrist_roll_joint"]
    (position, velocity, effort) = get_joint_states(joint_names)
    return position

def get_trajectory(joint_values):
    num_points = 1

    t = Float32MultiArray()
    t.layout.dim.append(MultiArrayDimension(
        size=num_points, stride=num_points*7, label="points"))
    t.layout.dim.append(MultiArrayDimension(size=7, stride=7, label="joints"))
    t.layout.data_offset = 0
    t.data = joint_values

    return t


def talker():
    rospy.init_node('trajectory')

    pub = rospy.Publisher('trajectory', Float32MultiArray, queue_size=1)

    rate = rospy.Rate(100)

    t2 = get_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    t1 = get_trajectory([-0.3853, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
              
    print(t1)
    # while not rospy.is_shutdown():
    #     pub.publish(t)
    #     rate.sleep()
    time.sleep(3)

    print get_joint_values()

    pub.publish(t1)

    time.sleep(1)

    print get_joint_values()

    pub.publish(t2)
    time.sleep(1)

    print get_joint_values()


if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
