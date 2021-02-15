import numpy as np
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
import pandas as pd
import matplotlib.pyplot as plt
from learn.RL.rl1_qtables_2 import VawtRLEnvironment
import vawt.physical_model.power_comparision_parameters as pm


blade = vb.VawtBlade(pm.params["blade_chord_length"], pm.params["airfoil_dir"], pm.params["blade_shaft_dist"])

pitch_change_cost_coef = 0.02
pitch_change_width = 70
sb_work = []

for tsr in pm.params['tsrs']:
    winds = []
    for wind_speed in pm.params['wind_speeds']:
        # find the best fixed angle for given wind and tsr
        # create environment
        # TSR = ωR/V∞
        rotor_speed = tsr * wind_speed / blade.rotor_radius
        env = VawtRLEnvironment(blade, pm.params["wind_direction"], wind_speed, rotor_speed, pm.params["theta_resolution"],
                                pm.params["pitch_resolution"], pitch_change_cost_coef, pitch_change_width)
        # get the pitch that gives maximum tangential force
        fp_work_polar = env.data.sum()
        max_work_pitch = fp_work_polar.idxmax()
        max_work = fp_work_polar.max()
        # save it
        winds.append(max_work)
    sb_work.append(winds)
    print("tsr", tsr)

df = pd.DataFrame(sb_work, index=pm.params['tsrs'], columns=pm.params['wind_speeds'])
df.to_csv(pm.params['fp_work_filename'])

xx, yy = np.meshgrid(df.index.values, df.columns.values)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.title.set_text('fp_work')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
ax.set_xlabel('tsr')
ax.set_ylabel('wind_speed')
ax.azim = -150
ax.elev = 88
ax.plot_surface(xx, yy, np.transpose(df), rstride=1, cstride=1, cmap='viridis', edgecolor='none')
plt.show()
