from scipy import interpolate
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import pandas as pd
from learn.airfoil_model.data_load import *

dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'

dataset = load_airfol_polar_from_dir(dir_path, fill_aoa=True)

X = dataset[['Re_number', 'aoa']]
Y = dataset[['cl', 'cd']]

# Plot using `.trisurf()`:
fig1 = plt.figure()
ax = fig1.add_subplot(121, projection='3d')
ax.title.set_text('oryginal_data')
ax.plot_trisurf(dataset['Re_number'], dataset['aoa'], dataset['cl'], cmap=cm.jet, linewidth=0.5)

# interpolation
x = dataset['aoa'].unique()
y = dataset['Re_number'].unique()
z = dataset['cl'].values.reshape(y.size, x.size)


xx, yy = np.meshgrid(x, y)

f = interpolate.interp2d(x, y, z, kind='cubic')
znew = f(x, y)

ax = fig1.add_subplot(122, projection='3d')
ax.title.set_text('interpolation')
ax.plot_surface(xx, yy, znew, rstride=1, cstride=1, cmap='viridis', edgecolor='none')

plt.show()

