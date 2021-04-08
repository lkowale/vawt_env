#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math


# subscriber
def state_sub_callback(msg):
    for blade in [1, 2]:
        err = msg.effort[blade] - msg.position[blade]
        # translate index from joint index to err_pub list index
        blade = blade -1
        err_pub[blade].publish(err)


rospy.init_node('pitch_error', anonymous=True)
speed_sub = rospy.Subscriber('joint_states', JointState, state_sub_callback, queue_size=1)

err_pub = [rospy.Publisher('/vawt_1/blade_1_err', Float32, queue_size=10),
             rospy.Publisher('/vawt_1/blade_2_err', Float32, queue_size=10)]

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    rate.sleep()
