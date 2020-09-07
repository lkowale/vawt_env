#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import math

# get current direction and speed from joint_state_publisher

speed = 0    # initialize value of global variable
direction = 0


# subscriber
def sub_callback(ros_data):
    global speed
    global direction
    speed = ros_data.position[ros_data.name.index('wind_speed')]
    direction = ros_data.position[ros_data.name.index('wind_direction')]


rospy.init_node('wind_simulator', anonymous=True)
sub = rospy.Subscriber('/joint_states', JointState, sub_callback, queue_size=1)
pub_speed = rospy.Publisher('/wind/speed', Float32, queue_size=10)
pub_direction = rospy.Publisher('/wind/direction', Float32, queue_size=10)
rate = rospy.Rate(20)

while not rospy.is_shutdown():
    pub_speed.publish(speed)
    pub_direction.publish(direction)
    rate.sleep()
