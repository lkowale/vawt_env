#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
# from tf.transformations import quaternion_from_euler
import math
# get current direction and speed from joint_state_publisher
from pyquaternion import Quaternion


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math


class RotorModel:
# keeeps info about: blades positions offset from rotor 0 position, first blade is on rotor 0 posiiotn
#                    blade aerodymic profile
# returns blades visualization vectors
    def __init__(self):
        self.blades = [
            # offset, pitch
            {'offset':0, 'pitch':0},
            {'offset':math.pi, 'pitch':0}
        ]
        self.theta = 0
        # airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
        self.blade = vb.VawtBlade(0.2, airfoil_dir, 1)

    def get_blade_markers(self, wind_vec, rotor_speed, theta, pitch):
        return self.blade.get_visualization_vectors(wind_vec, rotor_speed, theta, pitch)

    def get_blades_markers(self, wind_vec, rotor_speed):
        blades_vectors = {}
        for index, blade in enumerate(self.blades):
            blades_vectors[str(index)] = self.get_blade_markers(wind_vec, rotor_speed, self.theta + blade['offset'],
                                                                blade['pitch'])
        return blades_vectors

class RotorSimulator:
    def __init__(self):
        self.model = RotorModel()
        self.wind_vec = None
        self.wind_sub = rospy.Subscriber('/rotor/speed', Float32, self.speed_sub_callback, queue_size=1)
        # todo compose message for wind vector
        self.rotor_speed = 0
        self.rotor_speed_sub = rospy.Subscriber('/rotor/speed', Float32, self.speed_sub_callback, queue_size=1)

    def speed_sub_callback(self, ros_data):
        self.rotor_speed = ros_data.data



class BladeFrocesMarkers:

    def __init__(self):
        self.rotor_speed = 0
        self.rotor_position = 0
        rospy.init_node('forces_markers', anonymous=True)

        self.direction_sub = rospy.Subscriber('/rotor/position', Float32, self.position_sub_callback, queue_size=1)
        self.tangent_speed_marker_pub = rospy.Publisher('/rotor/marker/tangent_speed', Marker, queue_size=10)
        self.rate = rospy.Rate(20)
        self.airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        # airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
        # start the loop
        self.loop()

    def speed_sub_callback(self, ros_data):
        self.rotor_speed = ros_data.data

    def position_sub_callback(self, ros_data):
        self.rotor_position = ros_data.data

    def loop(self):
        while not rospy.is_shutdown():
            for blade in
            # marker = self.get_tangent_speed_marker()
            # self.tangent_speed_marker_pub.publish(marker)
            # self.rate.sleep()



    def compose_marker_vector(self, vector):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = vector.r
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
        return marker



