import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc

# plots tangential force in function of aoa and rotora blade theta
wind_direction = 0
wind_speed = 10

wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
# rotor_speed = 0.0001
rotor_speed = 20
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

blade = vb.VawtBlade(0.2, airfoil_dir, 1)

theta_range = [x * math.tau / 360 for x in range(-180, 180, 5)]
pitch_range = [x * math.tau / 360 for x in range(-180, 180, 5)]

# theta_range = [x*math.tau/360 for x in range(-180, 179, 5)]
# theta_range = [x*math.tau/360 for x in range(-540, 540, 10)]
# pitch_range = [x*math.tau/360 for x in range(-180, 179, 5)]
# pitch_range = [x*math.tau/360 for x in range(-90, 90, 5)]
# pitch_range = [x*math.tau/360 for x in range(-20, 20, 1)]
# pitch_range = [x*math.tau/360 for x in range(-60, 60, 1)]

# theta_range = [x for x in range(0, 10, 1)]
# pitch_range = [x for x in range(-8, 7, 1)]

thetas = []
for theta in theta_range:
    theta_ct_polar = [blade.get_normal_force(wind_vector, rotor_speed, theta, pitch) for pitch in pitch_range]
    thetas.append(theta_ct_polar)

df = pd.DataFrame(thetas, index=theta_range, columns=pitch_range)

xx, yy = np.meshgrid(theta_range, pitch_range)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.title.set_text('normal force')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
ax.set_xlabel('theta')
ax.set_ylabel('pitch')
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')

plt.show()
