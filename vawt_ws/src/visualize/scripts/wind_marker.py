#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
# from tf.transformations import quaternion_from_euler
import math
# get current direction and speed from joint_state_publisher
from pyquaternion import Quaternion

speed = 0    # initialize value of global variable
direction = 0


# subscriber
def speed_sub_callback(ros_data):
    global speed
    speed = ros_data.data


def direction_sub_callback(ros_data):
    global direction
    direction = ros_data.data


rospy.init_node('wind_marker', anonymous=True)
speed_sub = rospy.Subscriber('/wind/speed', Float32, speed_sub_callback, queue_size=1)
direction_sub = rospy.Subscriber('/wind/direction', Float32, direction_sub_callback, queue_size=1)
pub = rospy.Publisher('/wind/marker', Marker, queue_size=10)
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    marker = Marker()
    marker.header.frame_id = "/base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = speed / 20.0
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    q1 = Quaternion(axis=[0, 0, 1], angle=direction)
    q2 = Quaternion(axis=[0, 1, 0], angle=math.pi)
    q = q1 * q2
    # print
    # "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
    q = q.elements

    marker.pose.orientation.w = q[0]
    marker.pose.orientation.x = q[1]
    marker.pose.orientation.y = q[2]
    marker.pose.orientation.z = q[3]

    # marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q[0], q[1], q[2], q[3]
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 2.2
    pub.publish(marker)
    rate.sleep()
