import numpy as np

params = {
            'airfoil_dir': '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360',
            'op_interp_dir': '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_m_7/',
            'blade_shaft_dist': 1,
            'blade_chord_length': 0.2,
            'pitch_resolution': 4,
            'theta_resolution': 5,
            'wind_direction': 0,
            'wind_speeds': np.arange(3, 12),
            'tsrs': [0.1, 0.3, 0.5, 1, 1.5, 2, 3, 4, 5, 6],
            # 'wind_speeds': np.arange(3, 5),
            # 'tsrs': [6],
            'fp_work_filename': 'fp_work.csv',
            'vp_work_filename': 'vp_work.csv'
        }