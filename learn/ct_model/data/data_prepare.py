import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math

# from given airfoil directory create dataframe of tangential force
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

# variable ranges
# theta_range = [x*math.tau/360 for x in range(-180, 179, 30)]
# pitch_range = [x*math.tau/360 for x in range(-60, 60, 10)]
# wind_speed_range = [x for x in range(3, 20, 3)]
# rotor_speed_range = [x + 0.001 for x in range(0, 20, 3)]

# variable ranges
theta_range = [x*math.tau/360 for x in range(-180, 179, 10)]
pitch_range = [x*math.tau/360 for x in range(-60, 60, 5)]
wind_speed_range = [x for x in range(3, 20, 2)]
rotor_speed_range = [x + 0.001 for x in range(0, 20, 3)]

# given blade  VawtBlade(chord_length, airfoil_dir, rotor_radius)
blade = vb.VawtBlade(0.2, airfoil_dir, 1)

all_ct = pd.DataFrame()
for rotor_speed in rotor_speed_range:
    for wind_speed in wind_speed_range:
        wind_vector = vec.Vector2(r=wind_speed, theta=0)
        for theta in theta_range:
            theta_ct_polar = [blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in
                              pitch_range]
            df = pd.DataFrame()
            df['ct'] = theta_ct_polar
            df['pitch'] = pitch_range
            df['theta'] = theta
            df['wind_speed'] = wind_speed
            df['rotor_speed'] = rotor_speed
            all_ct = pd.concat([all_ct, df])

all_ct.to_csv('tangential_force.csv', index=False)


