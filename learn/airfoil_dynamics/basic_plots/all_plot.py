import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
from learn.airfoil_dynamics.ct_plot.base_calculus import *

# plots tangential force in function of aoa and rotora blade theta
wind_direction = 0
wind_speed = 3

wind_vector = get_wind_vector(wind_direction, wind_speed)
# rotor_speed = 0.0001
rotor_speed = 6
airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

blade = vb.VawtBlade(0.2, airfoil_dir, 1)

# theta_range = [x*math.tau/360 for x in range(-180, 179, 30)]
theta_range = [x*math.tau/360 for x in range(-180, 179, 5)]
# pitch_range = [x*math.tau/360 for x in range(-180, 179, 5)]
# pitch_range = [x*math.tau/360 for x in range(-90, 90, 5)]
pitch_range = [x*math.tau/360 for x in range(-60, 60, 5)]
# pitch_range = [x*math.tau/360 for x in range(-45, 45, 5)]

ft = []
blade_tangent_vector_r = []
blade_tangent_vector_theta = []
rel_wind_r = []
rel_wind_theta = []
re_number = []
for theta in theta_range:
    ft_p = []
    blade_tangent_vector_r_p = []
    blade_tangent_vector_theta_p = []
    rel_wind_r_p = []
    rel_wind_theta_p = []
    re_number_p = []
    for pitch in pitch_range:
        ft_s, fl_s, fd_s, cl_s, cd_s, aoa_rad_s, aoa_360, blade_chord_vector_s, re_number_s, rel_wind_s, blade_tangent_vector_s = \
            blade.get_tangential_force_talker(wind_vector, rotor_speed, theta, pitch)
        ft_p.append(ft_s)
        blade_tangent_vector_r_p.append(blade_tangent_vector_s.r)
        blade_tangent_vector_theta_p.append(blade_tangent_vector_s.theta)
        rel_wind_r_p.append(rel_wind_s.r)
        rel_wind_theta_p.append(rel_wind_s.theta)
        re_number_p.append(re_number_s)
    ft.append(ft_p)
    blade_tangent_vector_r.append(blade_tangent_vector_r_p)
    blade_tangent_vector_theta.append(blade_tangent_vector_theta_p)
    rel_wind_r.append(rel_wind_r_p)
    rel_wind_theta.append(rel_wind_theta_p)
    re_number.append(re_number_p)

df_ft = pd.DataFrame(ft, index=theta_range, columns=pitch_range)
df_btv_r = pd.DataFrame(blade_tangent_vector_r, index=theta_range, columns=pitch_range)
df_btv_theta = pd.DataFrame(blade_tangent_vector_theta, index=theta_range, columns=pitch_range)
df_rw_r = pd.DataFrame(rel_wind_r, index=theta_range, columns=pitch_range)
df_rw_theta = pd.DataFrame(rel_wind_theta, index=theta_range, columns=pitch_range)
df_re_number = pd.DataFrame(re_number, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)

fig = plt.figure()
ax = fig.add_subplot(231, projection='3d')
ax.title.set_text('ft')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df_ft), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax = fig.add_subplot(232, projection='3d')
ax.title.set_text('df_btv_r')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df_btv_r), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax = fig.add_subplot(233, projection='3d')
ax.title.set_text('df_re_number')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df_re_number), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax = fig.add_subplot(234, projection='3d')
ax.title.set_text('df_btv_theta')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df_btv_theta), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax = fig.add_subplot(235, projection='3d')
ax.title.set_text('rel_wind_r')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df_rw_r), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax = fig.add_subplot(236, projection='3d')
ax.title.set_text('rel_wind_theta')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df_rw_theta), rstride=1, cstride=1, cmap='viridis', edgecolor='none')


plt.show()
a=1
b=1
