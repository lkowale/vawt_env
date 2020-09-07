#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import math


class RotorSensorsSimulator:

    def __init__(self):
        self.rotor_speed = 0
        self.rotor_position = 0
        rospy.init_node('rotor_simulator', anonymous=True)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.sub_callback, queue_size=1)
        self.pub_speed = rospy.Publisher('/rotor/speed', Float32, queue_size=10)
        self.pub_position = rospy.Publisher('/rotor/position', Float32, queue_size=10)
        self.rate = rospy.Rate(20)
        # start the loop
        self.loop()

    # subscriber
    def sub_callback(self, ros_data):
        self.rotor_speed = ros_data.position[ros_data.name.index('rotor_speed')]
        self.rotor_position = ros_data.position[ros_data.name.index('main_shaft_joint')]

    def loop(self):
        while not rospy.is_shutdown():
            self.pub_speed.publish(self.rotor_speed)
            self.pub_position.publish(self.rotor_position)
            self.rate.sleep()

rss = RotorSensorsSimulator()

# if __name__ == '__main__':
#     main(sys.argv)
