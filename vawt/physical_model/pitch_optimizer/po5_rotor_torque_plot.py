#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt


class VawtTest:

    def __init__(self, physical_model, parameters):
        self.parameters = parameters
        self.physical_model = physical_model

    def blade_forces_polar(self, blade, rotor_speed, wind_speed):

        # TSR = ωR/V∞ -> w=TSR*wind_speed/R
        tsr = rotor_speed * blade.sa_radius / wind_speed
        wind_direction = 0
        wind_vec = self.physical_model.get_wind_vector(wind_direction, wind_speed)
        rotor_thetas = np.linspace(-np.pi, np.pi, num=100)
        opp_df = self.optimal_pitch_polar(tsr, blade, rotor_thetas)
        tf_df = [(theta, blade.vb.get_tangential_force(wind_vec, rotor_speed, theta, pitch)) for theta, pitch in opp_df]
        return opp_df, tf_df

    # return optimal pitch path for given tsr, blade
    def optimal_pitch_polar(self, tsr, blade, rotor_thetas):
        return [(rotor_theta, blade.get_optimal_pitch(tsr, rotor_theta, wind_direction=0)[0]) for rotor_theta in rotor_thetas]

    def plot_blades_optimal_pitch(self, tsr):
        rotor_thetas = np.linspace(-np.pi, np.pi, num=100)
        for blade in self.physical_model.blades:
            # get optimal pitch polar
            bop = pd.DataFrame(self.optimal_pitch_polar(tsr, blade, rotor_thetas), columns=['theta', 'op'])
            bop.plot(x='theta', y='op', kind='line')
            # get forces
            btf = self.tf_polar(tsr, blade, bop)
            btf.plot(x='theta', y='tf', kind='line')
        plt.show()

    def tf_polar(self, tsr, blade, optimal_pitch):
        tf = optimal_pitch
        rotor_speed = 3
        wind_speed = rotor_speed * blade.sa_radius / tsr
        wind_direction = 0
        wind_vec = self.physical_model.get_wind_vector(wind_direction, wind_speed)
        tf['tf'] = blade.vb.get_tangential_force(wind_vec, rotor_speed, tf['theta'], tf['op'])
        return tf


if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_3/'
    params = {
        'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
        # 'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360',
        'blade_shaft_dist': 1,
        'blade_chord_length': 0.2,
        'pitch_resolution': 4,
        'theta_resolution': 5,
        'wind_speed': 3,
        'wind_direction': 0,
        'tsr_start': 0.1,
        'tsr_stop': 7.0,
        'tsr_step': 0.3
    }
    twin_blades = [
        # blades_joint_name, chord_length, height, offset, sa_radius, airfoil_dir, optimal_path_dir
        pm.RotorBlade('blade_1_joint', 0.2, 2, 0, 1, airfoil_dir, op_interp_dir),
        pm.RotorBlade('blade_2_joint', 0.2, 2, np.pi, 1, airfoil_dir, op_interp_dir)
    ]
    vpm_tb = pm.VawtPhysicalModel(twin_blades)
    # gen plots of torque in function of rotor theta to check if there are any negative positions

    vt = VawtTest(vpm_tb, params)
    vt.plot_blades_optimal_pitch(1)
    # dfs = [pd.DataFrame(vt.blade_forces_polar(blade, 3, 3), columns=['theta', 't_force']) for blade in vpm_tb.blades]
    # # df = df.set_index('theta')
    # for df in dfs:
    #     df[0].plot(x='theta', y='op', kind='line')
    #     df[1].plot(x='theta', y='t_force', kind='line')
    # plt.show()
    # pass


