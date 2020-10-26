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
from vawt.physical_model.pitch_optimizer.po3a_interpolate import OptimalPathInterpolate
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3
import geometry_msgs
import vec


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
        # / vawt_1 / blade_1_joint_position_controller / command
        topic_name = "/vawt_1/" + self.joint_name + '_position_controller/command'
        self.publisher = rospy.Publisher(topic_name, Float64, queue_size=10)

    def get_tforce(self, wind_vec, rotor_speed, theta):
        tf = self.height * self.vb.get_tangential_force(wind_vec, rotor_speed, theta, self.pitch)
        return tf

    def get_torque(self, wind_vec, rotor_speed, rotor_theta):
        # calculate blade theta
        blade_theta = rotor_theta + self.offset
        # get tangential force and multiply by strut length
        return self.get_tforce(wind_vec, rotor_speed, blade_theta) * self.sa_radius


    def publish_command(self, command):
        self.publisher.publish(command)

    def get_optimal_pitch(self, tsr, rotor_theta, wind_direction):
        # get optimum pitch for current blade position
        blade_theta = rotor_theta + self.offset
        to_wind_theta = blade_theta - wind_direction
        return self.opi.get_optimal_pitch(tsr, to_wind_theta)



class VawtPhysicalModel:
    # subscribes to :
    #     /joint_states - OK
    #     /wind - TODO node for wind speed and direction publishing
    # publishes
    #   shaft_torque - OK
    #   blade_position_command

    def __init__(self, blades):


        # rotor speed musnt be equal 0 - for sake of relaitve wind vector - it cannot have 0 length
        self.blades = blades
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
        # shaft torque publisher
        self.shaft_torque_publisher = rospy.Publisher('/vawt_1/shaft_torque', Float32, queue_size=10)
        # gazebo apply wrench ros client
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        try:
            self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        except rospy.ServiceException:
            print("Service call failed")

    def joint_states_callback(self, data):
        # data.name = ['blade_1_joint', 'blade_2_joint', 'main_shaft_joint']
        self.theta = data.position[data.name.index('main_shaft_joint')]
        self.theta = vec.normalize_angle(self.theta)
        speed = data.velocity[data.name.index('main_shaft_joint')]
        if speed < 0.001:
            speed = 0.001
        self.speed = speed
        for blade in self.blades:
            blade.pitch = data.position[data.name.index(blade.joint_name)]

    def wind_speed_callback(self, data):
        speed = data.data
        if speed < 0.001:
            speed = 0.001
        self.wind_speed = speed

    def wind_direction_callback(self, data):
        self.wind_direction = data.data

    def step(self, d_time):
        # update wind vector
        self.wind_vec = self.get_wind_vector(self.wind_direction, self.wind_speed)
        self.shaft_torque = self.get_shaft_torque()
        # publish shaft torque
        self.shaft_torque_publisher.publish(self.shaft_torque)
        # use shaft torque to move gazebo model by calling gazebo service
        body_name = 'vawt_1::main_shaft'
        reference_frame = 'world'
        reference_point = geometry_msgs.msg.Point(x=0, y=0, z=0)
        wrench = geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(x=0, y=0, z=0),
                                          torque=geometry_msgs.msg.Vector3(x=0, y=0, z=self.shaft_torque))
        start_time = rospy.Time(secs=0, nsecs=0)
        duration = rospy.Duration(secs=d_time, nsecs=0)
        self.apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)

        # publish blades commands
        for blade in self.blades:
            tsr = self.speed * blade.sa_radius / self.wind_speed
            if tsr < 0.1:
                tsr = 0.1
            # get optimal pitch
            op = blade.get_optimal_pitch(tsr, self.theta, self.wind_direction)
            # translate it to joint position
            pos_comm = -op
            blade.publish_command(pos_comm)

    def get_blades_tforces(self, wind_vec):
        blades_tforces = []
        for blade in self.blades:
            blades_tforces.append(blade.get_tforce(wind_vec, self.speed, self.theta))
        return blades_tforces

    def get_shaft_torque(self):
        shaft_torque = 0
        for blade in self.blades:
            shaft_torque += blade.get_torque(self.wind_vec, self.speed, self.theta)
        return shaft_torque

    def get_wind_vector(self, wind_direction, wind_speed):
        return vb.get_wind_vector(wind_direction, wind_speed)


# make it to be launched as ROS node
if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_2/'

    twin_blades = [
        # chord_length, height, offset, sa_radius, airfoil_dir
        RotorBlade('blade_1_joint', 0.2, 2, 0, 1, airfoil_dir, op_interp_dir),
        RotorBlade('blade_2_joint', 0.2, 2, math.pi, 1, airfoil_dir, op_interp_dir)
    ]
    vpm = VawtPhysicalModel(twin_blades)
    rospy.init_node('VawtModel', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        vpm.step(0.02)
        rate.sleep()
