#!/usr/bin/env python
import roslib

import rospy
import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from vawt.physical_model.pitch_optimizer.po3a_interpolate import OptimalPathInterpolate


class RotorBlade:

    def __init__(self, joint_name, chord_length, height, offset, sa_radius, airfoil_dir, op_interpolator_dir):
        self.joint_name = joint_name
        self.height = height
        self.sa_radius = sa_radius
        self.vb = vb.VawtBlade(chord_length, airfoil_dir, sa_radius)
        self.opi = OptimalPathInterpolate(op_interpolator_dir)
        self.offset = offset
        self.pitch = 0
        # / vawt_1 / blade_1_position_controller / command
        # todo change controllers names so it can be easliyl converted to topic names
        # or extract blade anumber and use it to set topic name
        topic_name = "/vaw_1/" +
        self.publisher = rospy.Publisher('chatter', String, queue_size=10)

    def get_tforce(self, wind_vec, rotor_speed, theta):
        tf = self.height * self.vb.get_tangential_force(wind_vec, rotor_speed, theta, self.pitch)
        return tf

    def get_torque(self, wind_vec, rotor_speed, rotor_theta):
        # calculate blade theta
        blade_theta = rotor_theta + self.offset
        # get tangential force and multiply by strut length
        return self.get_tforce(wind_vec, rotor_speed, blade_theta) * self.sa_radius

    def publish_command(self, tsr, rotor_theta):



class VawtPhysicalModel:
    # subscribes to :
    #     /joint_states - OK
    #     /wind - TODO node for wind speed and direction publishing
    # publishes
    #   shaft_torque - OK
    #   blade_position_command

    def __init__(self, airfoil_dir):

        self.airfoil_dir = airfoil_dir
        self.blades = [
            # chord_length, height, offset, sa_radius, airfoil_dir
            RotorBlade('blade_1_joint', 0.2, 2, 0, 1, self.airfoil_dir),
            RotorBlade('blade_2_joint', 0.2, 2, math.pi, 1, self.airfoil_dir)
        ]
        # rotor speed musnt be equal 0 - for sake of relaitve wind vector - it cannot have 0 length
        self.speed = 0.001
        # theta is a position of a first blade
        self.theta = 0
        self.wind_direction = 0
        self.wind_speed = 0.001
        self.wind_vec = self.get_wind_vector(self.wind_direction, self.wind_speed)
        # self.wind_vec = None
        self.subscribers = [
            rospy.Subscriber('/vawt_1/joint_states', JointState, self.joint_states_callback),
            rospy.Subscriber('/vawt_1/wind/speed', Float32, self.wind_speed_callback),
            rospy.Subscriber('/vawt_1/wind/direction', Float32, self.wind_direction_callback)
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
        for blade in self.blades:
            blade.pitch = data.position[data.name.index(blade.joint_name)]


    def wind_states_callback(self, data):
        self.wind_vec = bc.get_wind_vector(data.direction, data.speed)

    def wind_speed_callback(self, data):
        self.wind_speed = data.data

    def wind_direction_callback(self, data):
        self.wind_direction = data.data

    def step(self, d_time):
        self.shaft_torque = self.get_shaft_torque()
        # publish shaft torque
        self.shaft_torque_publisher.publish(self.shaft_torque)
        # publish blades commands
        for blade in self.blades:
            blade.publish_command()

    def get_blades_tforces(self, wind_vec):
        blades_tforces = []
        for blade in self.blades:
            blades_tforces.append(blade.get_tforce(wind_vec, self.speed, self.theta))
        return blades_tforces

    def get_shaft_torque(self):
        shaft_torque = 0
        for blade in self.blades:
            shaft_torque += blade.get_torque()
        return shaft_torque

    def update(self, d_time):
        pass

    def get_wind_vector(self, wind_direction, wind_speed):
        return vb.get_wind_vector(wind_direction, wind_speed)

# make it to be launched as ROS node
if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/exps/naca0018_RL_1/'
    vpm = VawtPhysicalModel(airfoil_dir, op_interp_dir)
    rospy.init_node('VawtModel', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        vpm.step(0.02)
        rate.sleep()
