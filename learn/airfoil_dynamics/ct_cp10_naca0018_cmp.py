import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math

wind_vector = vec.Vector2(r=3, theta=0)
# rotor_speed = 0.0001
rotor_speed = 0.0001
cp10_airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
naca0018_airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

cp10_blade = vb.VawtBlade(0.2, cp10_airfoil_dir, 1)
naca0018_blade = vb.VawtBlade(0.2, naca0018_airfoil_dir, 1)

theta_range = [x*math.tau/360 for x in range(-180, 180, 10)]
# theta_range = [vec.normalize_angle(theta) for theta in theta_range]
pitch_range = [x*math.tau/360 for x in range(-90, 90, 10)]
# pitch_range = [x*math.tau/360 for x in range(-30, 30, 1)]

# theta_range = [x for x in range(0, 10, 1)]
# pitch_range = [x for x in range(-8, 7, 1)]

thetas = []
for theta in theta_range:
    theta_ct_polar = [cp10_blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.title.set_text('cp_10')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

# wind direction
wind_vector = vec.Vector2(r=3, theta=0)
thetas = []
for theta in theta_range:
    theta_ct_polar = [naca0018_blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)
ax = fig.add_subplot(122, projection='3d')
ax.title.set_text('naca0018_blade')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

##### rotor_speed
rotor_speed = 6
thetas = []
for theta in theta_range:
    theta_ct_polar = [cp10_blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)
df_cp10 = df
xx, yy = np.meshgrid(theta_range, pitch_range)
fig1 = plt.figure()
ax = fig1.add_subplot(121, projection='3d')
ax.title.set_text('cp_10')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

# wind direction
wind_vector = vec.Vector2(r=3, theta=math.pi / 2)
thetas = []
for theta in theta_range:
    theta_ct_polar = [naca0018_blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)
df_naca0018 = df
xx, yy = np.meshgrid(theta_range, pitch_range)
ax = fig1.add_subplot(122, projection='3d')
ax.title.set_text('naca0018_blade')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

# compare dataframes
# if df_cp10 == df_naca0018:
#     pass
df = pd.concat([df_cp10, df_naca0018])
df = df.reset_index(drop=True)
df_gpby = df.groupby(list(df.columns))
idx = [x[0] for x in df_gpby.groups.values() if len(x) == 1]
df.reindex(idx)

plt.show()

