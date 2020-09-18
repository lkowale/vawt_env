import numpy as np
import pandas as pd
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from keras.models import load_model
import matplotlib.pyplot as plt
from sklearn.pipeline import Pipeline
import joblib
from mpl_toolkits import mplot3d
# /home/aa/PycharmProjects/VGR 3d/model selection 11.06.2019_0/spatial_val_loss0.csv
# /home/aa/PycharmProjects/VGR 3d/model selection 11.06.2019_0/8_visualize_activ_func_comp.py
# /home/aa/PycharmProjects/VGR 3d/model selection 14.06.2019_0/cmp_act_val_loss.csv

# cross validation loss
filename = "/home/aa/vawt_env/learn/airfoil_model/cl_model/cl_compare_dense_val_loss.csv"
df = pd.read_csv(filename, index_col='model_name')
# drop time column
df.drop('elapsed_time', axis=1, inplace=True)
df = df.T
df.index = df.index.astype('int64')
val_loss = df

# show last 200 epochs
val_loss = val_loss.iloc[-200:]
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.title.set_text(filename.split('/')[-1])
for i, model in enumerate(val_loss.columns):
    ax.plot(np.full(val_loss.index.size, i), val_loss.index.values, val_loss[model])
plt.xticks(np.arange(val_loss.columns.size), val_loss.columns.values, rotation=90)

# cross learn loss
filename = "/home/aa/vawt_env/learn/airfoil_model/cl_model/cl_compare_dense_learn_loss.csv"
df = pd.read_csv(filename, index_col='model_name')
# drop time column
df.drop('elapsed_time', axis=1, inplace=True)
df = df.T
df.index = df.index.astype('int64')
val_loss = df

# show last 200 epochs
val_loss = val_loss.iloc[-200:]
ax = fig.add_subplot(122, projection='3d')
ax.title.set_text(filename.split('/')[-1])
for i, model in enumerate(val_loss.columns):
    ax.plot(np.full(val_loss.index.size, i), val_loss.index.values, val_loss[model])
plt.xticks(np.arange(val_loss.columns.size), val_loss.columns.values, rotation=90)

plt.show()