import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import time

from optimizer import Optimizer


def forward_kinematics_func(q):
    gfk = GetFK('l_wrist_roll_link', 'base_link')
    resp = gfk.get_current_fk(q)
        

def publisher():
    rospy.init_node('trajectory')

    pub = rospy.Publisher('trajectory', Float32MultiArray, queue_size=1)

    rate = rospy.Rate(100)

    num_points = 2
    
    t = Float32MultiArray()
    t.layout.dim.append(MultiArrayDimension(
        size=num_points, stride=num_points*7, label="points"))
    t.layout.dim.append(MultiArrayDimension(size=7, stride=7, label="joints"))
    t.layout.data_offset = 0
    t.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              -0.7853981633974483, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    print(t)
    # while not rospy.is_shutdown():
    #     pub.publish(t)
    #     rate.sleep()
    time.sleep(3)
    pub.publish(t)


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
