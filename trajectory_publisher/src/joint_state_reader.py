import time
import rospy
import roslib
import sys
roslib.load_manifest('joint_states_listener')
from joint_states_listener.srv import ReturnJointStates


def get_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


#pretty-print list to string
def pplist(list):
    return '\t'.join([str(x) for x in list])

def print_joint_values():
    joint_names = ["r_shoulder_pan_joint",
                   "r_shoulder_lift_joint",
                    "r_upper_arm_roll_joint",
                    "r_elbow_flex_joint",
                    "r_forearm_roll_joint",
                    "r_wrist_flex_joint",
                    "r_wrist_roll_joint"]
    (position, velocity, effort) = get_joint_states(joint_names)
    print "joint names:\t", pplist(joint_names)
    print "position:\t", pplist(position)
    print "velocity:\t", pplist(velocity)
    print "effort:\t", pplist(effort)
    time.sleep(1)
    

if __name__ == "__main__":
    print_joint_values()