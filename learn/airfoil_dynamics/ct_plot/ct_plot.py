import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math

# plots tangential force in function of aoa and rotor blade theta

wind_vector = vec.Vector2(r=3, theta=2)
# rotor_speed = 0.0001
rotor_speed = 0.0001
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

blade = vb.VawtBlade(0.2, airfoil_dir, 1)

theta_range = [x*math.tau/360 for x in range(-180, 179, 10)]
# theta_range = [vec.normalize_angle(theta) for theta in theta_range]
pitch_range = [x*math.tau/360 for x in range(-180, 179, 10)]

# theta_range = [x for x in range(0, 10, 1)]
# pitch_range = [x for x in range(-8, 7, 1)]

thetas = []
for theta in theta_range:
    theta_ct_polar = [blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)
fig = plt.figure()
ax = fig.add_subplot(131, projection='3d')
ax.title.set_text('tf wind r3t0')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

# wind direction
wind_vector = vec.Vector2(r=3, theta=math.pi / 2)
thetas = []
for theta in theta_range:
    theta_ct_polar = [blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)
ax = fig.add_subplot(132, projection='3d')
ax.title.set_text('tf wind r3tpi/2')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

# rotor speed
wind_vector = vec.Vector2(r=3, theta=0)
rotor_speed = 12
thetas = []
for theta in theta_range:
    theta_ct_polar = [blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)
ax = fig.add_subplot(133, projection='3d')
ax.title.set_text('tf wind r3tpi/2 rotor_speed')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

plt.show()
