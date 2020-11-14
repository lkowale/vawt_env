#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
from vawt.physical_model.pitch_optimizer.po5_rotor_tr_2b import VawtTest
from vawt.physical_model.pitch_optimizer.po2_generate_optimal_path import OptimalPath
from vawt.physical_model.pitch_optimizer.po1_generate_q_tables import PitchOptimizer
from vawt.physical_model.work_power import VawtPowerTest


if __name__ == '__main__':


    params = {
        'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
        # 'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360',
        'results_dir_prepend': 'tsr_test/',
        'results_dir_append': '_ws_/',
        'blade_shaft_dist': 1,
        'blade_chord_length': 0.2,
        'pitch_resolution': 4,
        'theta_resolution': 5,
        'wind_speed': 3,
        'wind_direction': 0,
        'tsr_start': 6.1,
        'tsr_stop': 6.9,
        'tsr_step': 0.2
    }

    tsr = 6.1
    run_list = [1, 2, 3]

    # 1 generate q-tables for given tsr but different wind speed
    for run in run_list:
        params['results_dir_prepend'] = 'same_params_test/'
        params['results_dir_append'] = '_ws_' + str(run) + '/'
        po = PitchOptimizer(params)
        po.gensave_q_tables()
    # 2 genereta optimal paths
    for run in run_list:
        op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/same_params_test/naca0018_ws_' + str(run) + '/')
        # op = OptimalPath('/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/cp10_RL_5/')
        op.find_optimum_params(save_plot=True)
    # # 3 plot torque polar
    # for run in run_list:
    #     op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/same_params_test/naca0018_ws_' + str(run) + '/'
    #     twin_blades = [
    #         # blades_joint_name, chord_length, height, offset, sa_radius, airfoil_dir, optimal_path_dir
    #         pm.RotorBlade('blade_1_joint', 0.2, 2, 0, 1, params['airfoil_dir'], op_interp_dir),
    #         pm.RotorBlade('blade_2_joint', 0.2, 2, np.pi, 1, params['airfoil_dir'], op_interp_dir)
    #     ]
    #     vpm_tb = pm.VawtPhysicalModel(twin_blades)
    #     # gen plots of torque in function of rotor theta to check if there are any negative positions
    #     vt = VawtTest(vpm_tb, params, 100)
    #     vt.plot_torque_polar()

    # 4 plot work
    for run in run_list:

        params['rotor_speed'] = tsr * params['wind_speed']
        params['op_interp_dir'] = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/same_params_test/naca0018_ws_' + str(run) + '/'
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
        vpt.plot_work(vpt.get_work_power())
    plt.show()
