import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc



# plots tangential force in function of aoa and rotora blade theta
wind_direction = 0
wind_speed = 3

wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
# rotor_speed = 0.0001
rotor_speed = 3
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

blade = vb.VawtBlade(0.2, airfoil_dir, 1)

theta_pitch = [
    (-2, 0),
    (1, 0),
    (-1, 20*math.tau/360),
    (-2, -2)
]
results = []
for theta, pitch in theta_pitch:
    results.append(blade.get_tangential_force_talker(wind_vector, rotor_speed, theta, pitch))

df = pd.DataFrame(results, columns=['ft_s', 'fl_s', 'fd_s', 'cl_s', 'cd_s', 'aoa_rad_s', 'aoa_360', 'blade_chord_vector_s', 're_number_s', 'rel_wind_s', 'blade_tangent_vector_s'])
df_tp = pd.DataFrame(theta_pitch, columns=['theta', 'pitch'])
df[['theta', 'pitch']] = df_tp[['theta', 'pitch']]
df.plot()
