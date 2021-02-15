#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
from vawt.physical_model.pitch_optimizer.po5_rotor_tr_2b import VawtTest
from matplotlib import cm
from vawt.physical_model.wind_power_interpolation import InterpolateWindPower
import vawt.physical_model.power_comparision_parameters as pam


class VawtPowerTest:

    def __init__(self, physical_model, parameters):
        self.params = parameters
        self.physical_model = physical_model
        self.wind_speed_range = self.params['wind_speeds']
        # self.tsr_range = np.arange(self.params['tsr_start'], self.params['tsr_stop'], self.params['tsr_step'])
        self.tsr_range = self.params['tsrs']
        self.wpi = InterpolateWindPower()
        self.swept_area = self.physical_model.blades[0].height * self.physical_model.blades[0].sa_radius * 2
        self.betz_limit = 0.593

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
                # if power exeedes power of wind stop counting
                if self.wpi.get_wind_power(wind_speed) * self.swept_area * self.betz_limit < power:
                    break
                rs_power.append(power)
            tsr_work.append(rs_work)
            tsr_power.append(rs_power)
            print("tsr", tsr)

        work_df = pd.DataFrame(tsr_work, index=self.tsr_range, columns=self.wind_speed_range).fillna(0)
        power_df = pd.DataFrame(tsr_power, index=self.tsr_range, columns=self.wind_speed_range).fillna(0)
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

    _blades = [
        # blades_joint_name, chord_length, height, offset, sa_radius, airfoil_dir, optimal_path_dir
        pm.RotorBlade('blade_1_joint', 0.2, 2, 0, pam.params['blade_shaft_dist'], pam.params['airfoil_dir'], pam.params['op_interp_dir']),
        pm.RotorBlade('blade_2_joint', 0.2, 2, 2 * np.pi / 3, pam.params['blade_shaft_dist'], pam.params['airfoil_dir'], pam.params['op_interp_dir']),
        pm.RotorBlade('blade_3_joint', 0.2, 2, -2 * np.pi / 3, pam.params['blade_shaft_dist'], pam.params['airfoil_dir'], pam.params['op_interp_dir'])
    ]
    vpm_tb = pm.VawtPhysicalModel(_blades)


    # gen plots of torque in function of rotor theta to check if there are any negative positions
    vpt = VawtPowerTest(vpm_tb, pam.params)
    # vt.plot_blades_op_tf()
    work, power = vpt.get_work_power()
    work.to_csv(pam.params['vp_work_filename'])
    vpt.plot_work_power(work, power)
    plt.show()
    # dfs = [pd.DataFrame(vt.blade_forces_polar(blade, 3, 3), columns=['theta', 't_force']) for blade in vpm_tb.blades]
    # # df = df.set_index('theta')
    # for df in dfs:
    #     df[0].plot(x='theta', y='op', kind='line')
    #     df[1].plot(x='theta', y='t_force', kind='line')
    # plt.show()
    # pass


