#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState

from tf import transformations
import numpy as np


"""
Class to make FK calls using the /compute_fk service.
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GetFK(object):
    def __init__(self, fk_link, frame_id):
        """
        A class to do FK calls thru the MoveIt!'s /compute_ik service.
        :param str fk_link: link to compute the forward kinematics
        :param str frame_id: frame_id to compute the forward kinematics
        into account collisions
        """
        rospy.loginfo("Initalizing GetFK...")
        self.fk_link = fk_link
        self.frame_id = frame_id
        rospy.loginfo("Asking forward kinematics for link: " + self.fk_link)
        rospy.loginfo("PoseStamped answers will be on frame: " + self.frame_id)
        self.fk_srv = rospy.ServiceProxy('/compute_fk',
                                         GetPositionFK)
        rospy.loginfo("Waiting for /compute_fk service...")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Connected!")
        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

    def js_cb(self, data):
        self.last_js = data

    def get_current_fk_pose(self):
        resp = self.get_current_fk()
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None

    def get_current_fk(self):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.logwarn("Waiting for a /joint_states message...")
            rospy.sleep(0.1)
        return self.get_fk(self.last_js)

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        """
        Do an FK call to with.
        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        if fk_link is None:
            fk_link = self.fk_link

        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [self.fk_link]
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp


def get_transformation_matrix(resp):
    position = resp.pose_stamped[0].pose.position
    orientation = resp.pose_stamped[0].pose.orientation

    rot_mat = transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    
    transform_mat = rot_mat
    transform_mat[:3, 3] = np.array([position.x, position.y, position.z])

if __name__ == '__main__':
    rospy.init_node('test_fk')
    rospy.loginfo("Querying for FK")
    gfk = GetFK('l_wrist_roll_link', 'base_link')
    resp = gfk.get_current_fk()
    print gfk.last_js

    # from moveit_python_tools.friendly_error_codes import moveit_error_dict
    # rospy.loginfo(moveit_error_dict[resp.error_code.val])
    rospy.loginfo(resp)
    # rospy.loginfo(dir(resp))
    
    pose = resp.pose_stamped[0].pose
    pose_stamped = resp.pose_stamped[0]
    position = pose.position
    orientation = pose.orientation
    print "position", position
    rot_mat = transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
    
    transform_mat = rot_mat
    transform_mat[:3, 3] = np.array([position.x, position.y, position.z])
    print(repr(transform_mat))

    
    from get_ik import GetIK
    rospy.loginfo("Querying for IK")
    gik = GetIK(group='right_arm')

    resp = gik.get_ik(pose_stamped)
    print resp
    # rospy.loginfo(dir(resp.pose_stamped[0].pose.position))
