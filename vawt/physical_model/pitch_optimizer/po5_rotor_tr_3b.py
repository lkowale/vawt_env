#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb

from vawt.physical_model.pitch_optimizer.po5_rotor_tr_2b import *


if __name__ == '__main__':
    # airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    # op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_4/'
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
    op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/cp10_RL_4/'
    params = {
        'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
        # 'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360',
        'blade_shaft_dist': 1,
        'blade_chord_length': 0.2,
        'pitch_resolution': 4,
        'theta_resolution': 5,
        'wind_speed': 3,
        'wind_direction': 0,
        'rotor_speed': 0.1,
        'tsr_start': 0.1,
        'tsr_stop': 7.0,
        'tsr_step': 0.3
    }
    _blades = [
        # blades_joint_name, chord_length, height, offset, sa_radius, airfoil_dir, optimal_path_dir
        pm.RotorBlade('blade_1_joint', 0.2, 2, 0, 1, airfoil_dir, op_interp_dir),
        pm.RotorBlade('blade_2_joint', 0.2, 2, 2*np.pi/3, 1, airfoil_dir, op_interp_dir),
        pm.RotorBlade('blade_3_joint', 0.2, 2, -2*np.pi/3, 1, airfoil_dir, op_interp_dir)
    ]
    vpm_tb = pm.VawtPhysicalModel(_blades)
    # gen plots of torque in function of rotor theta to check if there are any negative positions

    vt = VawtTest(vpm_tb, params, 100)
    # vt.plot_blades_op_tf()
    vt.plot_torque_polar()
    # dfs = [pd.DataFrame(vt.blade_forces_polar(blade, 3, 3), columns=['theta', 't_force']) for blade in vpm_tb.blades]
    # # df = df.set_index('theta')
    # for df in dfs:
    #     df[0].plot(x='theta', y='op', kind='line')
    #     df[1].plot(x='theta', y='t_force', kind='line')
    # plt.show()
    # pass


