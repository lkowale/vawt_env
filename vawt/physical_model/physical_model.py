#!/usr/bin/env python
import roslib

import rospy
import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class VawtBlade:

    def __init__(self, chord_length, height, offset, sa_radius, airfoil_dir):
        self.height = height
        self.sa_radius = sa_radius
        self.vb = vb.VawtBlade(chord_length, airfoil_dir, sa_radius)
        self.offset = offset
        self.pitch = 0


    def get_tforce(self, wind_vec, rotor_speed, rotor_theta):
        tf = self.height * self.vb.get_tangential_force(wind_vec, rotor_speed, rotor_theta, self.pitch)
        return tf


class VawtPhysicalModel:
    # subscribes to :
    #     /joint_states - OK
    #     /wind - TODO node for wind speed and direction publishing
    # publishes
    #   shaft_torque - OK
    #   blade_position_command

    def __init__(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
        self.blades = [
            # chord_length, height, offset, sa_radius, airfoil_dir
            VawtBlade(0.2, 2, 0, 1, airfoil_dir),
            VawtBlade(0.2, 2, math.pi/2, 1, airfoil_dir)
        ]
        self.speed = 0.001
        # theta is a position of a first blade
        self.theta = 0
        wind_direction = 0
        wind_speed = 6
        self.wind_vec = bc.get_wind_vector(wind_direction, wind_speed)
        # self.wind_vec = None
        self.subscribers = [
            rospy.Subscriber('/vawt_1/joint_states', JointState, self.joint_states_callback),
            rospy.Subscriber('/vawt_1/wind', JointState, self.wind_states_callback)
        ]
        self.shaft_torque_publisher = rospy.Publisher('/vawt_1/shaft_torque', Float32, queue_size=10)
        self.blade_position_publishers = []
        for i, blade in enumerate(self.blades):
            self.blade_position_publishers.append(rospy.Publisher('/vawt_1/blade_' + i + '/position_controller/command', Float32, queue_size=10))

    def joint_states_callback(self, data):
        # data.name = ['blade_1_joint', 'blade_2_joint', 'main_shaft_joint']
        self.theta = data.position[data.name.index('main_shaft_joint')]
        speed = data.velocity[data.name.index('main_shaft_joint')]
        if speed < 0.001:
            speed = 0.001
        self.speed = speed

    def wind_states_callback(self, data):
        self.wind_vec = bc.get_wind_vector(data.direction, data.speed)

    def step(self, d_time):
        # calculate blade forces
        blades_tforces = self.get_blades_tforces(self.wind_vec)
        # calculate shaft torque
        self.shaft_torque = self.get_shaft_torque(blades_tforces)
        # publish shaft torque
        self.shaft_torque_publisher.publish(self.shaft_torque)
        #
        # update rotor position
        self.update(d_time)


    def reset(self):
        # for sake of vec library speed of rotor cannot be = 0
        self.speed = 0.001
        self.theta = 0

    def get_blades_tforces(self, wind_vec):
        blades_tforces = []
        for blade in self.blades:
            blades_tforces.append(blade.get_tforce(wind_vec, self.speed, self.theta))
        return blades_tforces

    def get_shaft_torque(self, blades_tforces):
        for i, blade_tforce in enumerate(blades_tforces):
            shaft_torque = blade_tforce * self.blades[i].sa_radius
        return shaft_torque

    def update(self, d_time):
        pass


if __name__ == '__main__':
    vpm = VawtPhysicalModel()
    rospy.init_node('VawtModel', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        vpm.step(0.02)
        rate.sleep()
