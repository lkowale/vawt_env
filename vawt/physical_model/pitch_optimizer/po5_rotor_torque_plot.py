#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb


class TestBlade:

    def __init__(self, rot_blade, parameters, rotor_thetas):
        self.rotor_thetas = rotor_thetas
        self.parameters = parameters
        self.rot_blade = rot_blade
        # get optimal pitch polar
        # TSR = ωR/V∞ -> w=TSR*wind_speed/R
        tsr = self.parameters['rotor_speed'] * self.rot_blade.sa_radius / self.parameters['wind_speed']
        self.opp = self.optimal_pitch_polar(tsr, self.rot_blade, self.rotor_thetas, self.parameters['wind_direction'])
        self.btf = self.blade_forces_polar(self.opp, self.rot_blade, self.parameters['rotor_speed'], self.parameters['wind_speed'],
                                  self.parameters['wind_direction'])
    # return optimal pitch path for given tsr, blade, list_of_thetas as a lst of pairs (theta,pitch)
    def optimal_pitch_polar(self, tsr, blade, rotor_thetas, wind_direction):
        opp = [(rotor_theta, blade.get_optimal_pitch(tsr, rotor_theta, wind_direction)[0]) for rotor_theta in rotor_thetas]
        return pd.DataFrame(opp, columns=['theta', 'op'])

    def blade_forces_polar(self, pitch_polar, blade, rotor_speed, wind_speed, wind_direction):
        wind_vec = vb.get_wind_vector(wind_direction, wind_speed)
        tf_df = pitch_polar
        tf = []
        for index, row in tf_df.iterrows():
            # set blade pitch, it uses it to calculate tf
            blade.pitch = row['op']
            tf.append(blade.get_tforce(wind_vec, rotor_speed, row['theta']))
        tf_df['tf'] = tf
        return tf_df


class VawtTest:

    def __init__(self, physical_model, parameters):
        self.parameters = parameters
        self.physical_model = physical_model
        self.rotor_thetas = np.linspace(-np.pi, np.pi, num=100)
        self.test_blades = [TestBlade(blade, self.parameters, self.rotor_thetas) for blade in self.physical_model.blades]

    def plot_blades_op_tf(self):

        for blade in self.test_blades:
            blade.opp.plot(x='theta', y='op', kind='line')
            # get forces
            blade.btf.plot(x='theta', y='tf', kind='line')
        plt.show()

    def plot_torque_polar(self):
        torque = pd.DataFrame()
        torque['theta'] = self.rotor_thetas
        torque['trq'] = 0
        for blade in self.test_blades:
            torque['trq'] += blade.btf['tf'] * blade.rot_blade.sa_radius
        torque.plot(x='theta', y='trq', kind='line')
        plt.show()

if __name__ == '__main__':
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_4/'
    params = {
        'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
        # 'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360',
        'blade_shaft_dist': 1,
        'blade_chord_length': 0.2,
        'pitch_resolution': 4,
        'theta_resolution': 5,
        'wind_speed': 3,
        'wind_direction': 0,
        'rotor_speed': 0.3,
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
    vt.plot_blades_op_tf()
    vt.plot_torque_polar()
    # dfs = [pd.DataFrame(vt.blade_forces_polar(blade, 3, 3), columns=['theta', 't_force']) for blade in vpm_tb.blades]
    # # df = df.set_index('theta')
    # for df in dfs:
    #     df[0].plot(x='theta', y='op', kind='line')
    #     df[1].plot(x='theta', y='t_force', kind='line')
    # plt.show()
    # pass


