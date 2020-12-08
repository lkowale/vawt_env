#!/usr/bin/env python
import roslib

import rospy
import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from vawt.physical_model.physical_model import RotorBlade
from vawt.physical_model.physical_model import VawtPhysicalModel
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3
import geometry_msgs
import vec


class RotorBladeROSinterface:

    def __init__(self, blade):
        # / vawt_1 / blade_1_position_controller / command
        # / vawt_1 / blade_1_joint_position_controller / command
        self.blade = blade
        topic_name = "/vawt_1/" + self.blade.joint_name + '_position_controller/command'
        self.publisher = rospy.Publisher(topic_name, Float64, queue_size=10)

    def publish_command(self, command):
        self.publisher.publish(command)


class VawtOptimalController:

    def __init__(self, vpm):
        self.vpm = vpm
        self.ros_blades = [RotorBladeROSinterface(blade) for blade in self.vpm.blades]

        self.subscribers = [
            rospy.Subscriber('/vawt_1/joint_states', JointState, self.joint_states_callback),
            rospy.Subscriber('/vawt_1/wind/speed', Float32, self.wind_speed_callback),
            rospy.Subscriber('/vawt_1/wind/direction', Float32, self.wind_direction_callback)
        ]

    def joint_states_callback(self, data):
        # data.name = ['blade_1_joint', 'blade_2_joint', 'main_shaft_joint']
        theta = data.position[data.name.index('main_shaft_joint')]
        self.vpm.theta = vec.normalize_angle(theta)
        speed = data.velocity[data.name.index('main_shaft_joint')]
        if speed < 0.001:
            speed = 0.001
        self.vpm.speed = speed
        for ros_blade in self.ros_blades:
            ros_blade.blade.pitch = data.position[data.name.index(ros_blade.blade.joint_name)]

    def wind_speed_callback(self, data):
        speed = data.data
        if speed < 0.001:
            speed = 0.001
        self.vpm.wind_speed = speed

    def wind_direction_callback(self, data):
        self.vpm.wind_direction = data.data

    def step(self, d_time):
        # do not do anything unless wind speed is more than 3m/s
        if self.vpm.wind_speed > 3:
            # update wind vector
            self.vpm.wind_vec = self.vpm.get_wind_vector(self.vpm.wind_direction, self.vpm.wind_speed)
            self.vpm.shaft_torque = self.vpm.get_shaft_torque()

            # publish blades commands
            for ros_blade in self.ros_blades:
                tsr = self.vpm.speed * ros_blade.blade.sa_radius / self.vpm.wind_speed
                if tsr < 0.1:
                    tsr = 0.1
                # get optimal pitch
                op = ros_blade.blade.get_optimal_pitch(self.vpm.wind_speed, tsr, self.vpm.theta, self.vpm.wind_direction)
                if op == 0:
                    pass
                # translate it to joint position
                ros_blade.publish_command(op)


# make it to be launched as ROS node
if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_m_7/'
    # airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
    # op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/cp10_RL_4/'
    twin_blades = [
        # chord_length, height, offset, sa_radius, airfoil_dir
        RotorBlade('blade_1_joint', 0.2, 2, 0, 1, airfoil_dir, op_interp_dir),
        RotorBlade('blade_2_joint', 0.2, 2, math.pi, 1, airfoil_dir, op_interp_dir)
    ]

    r_vpm = VawtOptimalController(VawtPhysicalModel(twin_blades))
    rospy.init_node('VawtOptimalController', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        r_vpm.step(0.02)
        rate.sleep()
