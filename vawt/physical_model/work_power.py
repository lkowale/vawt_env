#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
from vawt.physical_model.pitch_optimizer.po5_rotor_tr_2b import VawtTest
from matplotlib import cm


class VawtPowerTest:

    def __init__(self, physical_model, parameters):
        self.params = parameters
        self.physical_model = physical_model
        self.wind_speed_range = self.params['wind_speeds']
        # self.tsr_range = np.arange(self.params['tsr_start'], self.params['tsr_stop'], self.params['tsr_step'])
        self.tsr_range = self.params['tsrs']

    # 'wind_speed': 3,
    # 'wind_direction': 0,
    # 'rotor_speed': 0.1,
    def get_work_power(self):
        tsr_work = []
        tsr_power = []
        for tsr in self.tsr_range:
            rs_work = []
            rs_power = []
            for wind_speed in self.wind_speed_range:
                self.params['wind_speed'] = wind_speed
                self.params['rotor_speed'] = wind_speed*tsr
                vt = VawtTest(self.physical_model, self.params, 100)
                work = vt.work_per_revolution()
                rs_work.append(work)
                power = work/(math.tau/self.params['rotor_speed'])
                rs_power.append(power)
            tsr_work.append(rs_work)
            tsr_power.append(rs_power)
            print("tsr", tsr)

        work_df = pd.DataFrame(tsr_work, index=self.tsr_range, columns=self.wind_speed_range)
        power_df = pd.DataFrame(tsr_power, index=self.tsr_range, columns=self.wind_speed_range)
        return work_df, power_df

    def plot_work_power(self, work_df, power_df):
        # interpolation
        # tsr
        x = self.tsr_range
        # theta
        y = self.wind_speed_range
        xx, yy = np.meshgrid(x, y)
        z = np.transpose(work_df)
        fig = plt.figure()
        ax = fig.add_subplot(121, projection='3d')
        ax.plot_surface(xx, yy, z, rstride=1, cstride=1, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)
        ax.title.set_text('Work done in one revolution')
        ax.set_xlabel('tsr')
        ax.set_ylabel('wind speed')
        ax.set_zlabel('Work')

        z = np.transpose(power_df)
        ax = fig.add_subplot(122, projection='3d')
        ax.plot_surface(xx, yy, z, rstride=1, cstride=1, cmap=cm.coolwarm,
                               linewidth=0, antialiased=False)
        # ax.title.set_text('Power')
        ax.set_xlabel('tsr')
        ax.set_ylabel('wind speed')
        ax.set_zlabel('Power')


if __name__ == '__main__':

    params = {
            'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
            'op_interp_dir': '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_m_7/',
            # 'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360',
            # 'op_interp_dir': '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/cp10_RL_4/',
            'blade_shaft_dist': 1,
            'blade_chord_length': 0.2,
            'pitch_resolution': 4,
            'theta_resolution': 5,
            'wind_direction': 0,
            'wind_speeds': np.arange(3, 12),
            # 'wind_speeds': np.arange(10, 11),
            'tsrs': [0.1, 0.3, 0.5, 1, 1.5, 2, 3, 4, 5, 6],
            # 'tsrs': [2]
        }
    _blades = [
        # blades_joint_name, chord_length, height, offset, sa_radius, airfoil_dir, optimal_path_dir
        pm.RotorBlade('blade_1_joint', 0.2, 2, 0, params['blade_shaft_dist'], params['airfoil_dir'], params['op_interp_dir']),
        pm.RotorBlade('blade_2_joint', 0.2, 2, 2 * np.pi / 3, params['blade_shaft_dist'], params['airfoil_dir'], params['op_interp_dir']),
        pm.RotorBlade('blade_3_joint', 0.2, 2, -2 * np.pi / 3, params['blade_shaft_dist'], params['airfoil_dir'], params['op_interp_dir'])
    ]
    vpm_tb = pm.VawtPhysicalModel(_blades)


    # gen plots of torque in function of rotor theta to check if there are any negative positions
    vpt = VawtPowerTest(vpm_tb, params)
    # vt.plot_blades_op_tf()
    vpt.plot_work_power(*vpt.get_work_power())
    plt.show()
    # dfs = [pd.DataFrame(vt.blade_forces_polar(blade, 3, 3), columns=['theta', 't_force']) for blade in vpm_tb.blades]
    # # df = df.set_index('theta')
    # for df in dfs:
    #     df[0].plot(x='theta', y='op', kind='line')
    #     df[1].plot(x='theta', y='t_force', kind='line')
    # plt.show()
    # pass


