import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math

# plots tangential force in function of aoa and rotora blade theta

wind_vector = vec.Vector2(r=3, theta=2)
# rotor_speed = 0.0001
rotor_speed = 0.0001
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

blade = vb.VawtBlade(0.2, airfoil_dir, 1)

reynolds_range = [x*10000 for x in range(10, 100, 10)]
# theta_range = [vec.normalize_angle(theta) for theta in theta_range]
aoa_range = [x for x in range(-180, 179, 10)]

cl_list = []
cd_list = []
for re_num in reynolds_range:
    cl_polar = [blade.get_coeffs(aoa_360, re_num)[0] for aoa_360 in aoa_range]
    cd_polar = [blade.get_coeffs(aoa_360, re_num)[1] for aoa_360 in aoa_range]
    cl_list.append(cl_polar)
    cd_list.append(cd_polar)

df_cl = pd.DataFrame(cl_list, index=reynolds_range, columns=aoa_range)
df_cd = pd.DataFrame(cd_list, index=reynolds_range, columns=aoa_range)

xx, yy = np.meshgrid(reynolds_range, aoa_range)
fig = plt.figure()

ax = fig.add_subplot(121, projection='3d')
ax.title.set_text('df_cl')
ax.set_xlabel('reynolds_range')
ax.set_ylabel('aoa_range')
ax.plot_surface(xx, yy, np.transpose(df_cl), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax = fig.add_subplot(122, projection='3d')
ax.title.set_text('df_cd')
ax.set_xlabel('reynolds_range')
ax.set_ylabel('aoa_range')
ax.plot_surface(xx, yy, np.transpose(df_cd), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

plt.show()


