import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc


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

    def __init__(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
        self.blades = [
            # chord_length, offset, sa_radius, airfoil_dir
            VawtBlade(0.2, 0, 1, airfoil_dir),
            VawtBlade(0.2, math.pi/2, 1, airfoil_dir)
        ]
        self.speed = 0.001
        self.theta = 0



    def step(self, d_time, wind_vec):
        # calculate blade forces
        blades_tforces = self.get_blades_tforces(wind_vec)
        # calculate shaft torque
        shaft_torque = self.get_shaft_torque(blades_tforces)
        # calculate shaft speed
        self.speed = self.get_rotor_speed(shaft_torque)
        # update rotor position
        self.update(d_time)

        return blades_tforces, shaft_torque

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

    def get_rotor_speed(self, shaft_torque):
        pass