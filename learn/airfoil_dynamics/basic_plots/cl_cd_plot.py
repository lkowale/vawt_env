from os import listdir
from os.path import isfile, join
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from learn.airfoil_model.data_load import *

# plots lift and drag coefficient in function of Reynolds number and angle of attack

airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
# airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
cl_cd_df = load_airfol_polar_from_dir(airfoil_dir, fill_aoa=True)
cl_cd_df = cl_cd_df.set_index(['aoa', 'Re_number']).unstack(level=-1)

cl_df = cl_cd_df['cl']
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.title.set_text('cl')
for i, model in enumerate(cl_df.columns):
    ax.plot(np.full(cl_df.index.size, i), cl_df.index.values, cl_df[model])
plt.xticks(np.arange(cl_df.columns.size), cl_df.columns.values, rotation=90)

cd_df = cl_cd_df['cd']
ax = fig.add_subplot(122, projection='3d')
ax.title.set_text('cd')
for i, model in enumerate(cd_df.columns):
    ax.plot(np.full(cd_df.index.size, i), cd_df.index.values, cd_df[model])
plt.xticks(np.arange(cd_df.columns.size), cd_df.columns.values, rotation=90)



plt.show()


