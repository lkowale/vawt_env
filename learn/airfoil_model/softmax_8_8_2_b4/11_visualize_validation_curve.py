from sklearn.model_selection import validation_curve
import numpy as np
from matplotlib import pyplot as plt
import numpy
import pandas as pd
import time
from keras.models import Sequential
from keras import regularizers
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from keras import regularizers
from keras.layers import Dense, Dropout, Activation
from learn.airfoil_model.data_load import *

dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'

dataset = load_airfol_polar_from_dir(dir_path)

X = dataset[['Re_number', 'aoa']]
Y = dataset[['cl', 'cd']]

start_time = time.time()
def model_base():
    # create model
    model = Sequential()
    model.add(Dense(8, input_dim=2, kernel_initializer='normal', activation='relu'))
    model.add(Dense(8, kernel_initializer='normal', activation='relu'))
    model.add(Dense(2, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model


# fix random seed for reproducibility
seed = 2018
numpy.random.seed(seed)

x_scaler = StandardScaler()
y_scaler = StandardScaler()
X_scaled = x_scaler.fit_transform(X)
Y_scaled = y_scaler.fit_transform(Y)

# Fit the model
history = model_base().fit(X_scaled, Y_scaled, validation_split=0.25, epochs=100, batch_size=4, verbose=1)
# list all data in history
print(history.history.keys())

# summarize history MAE
plt.figure(1)
plt.plot(history.history['mae'])
plt.plot(history.history['val_mae'])
plt.title('mean_absolute_error')
plt.ylabel('mean_absolute_error')
plt.xlabel('epoch')
plt.legend(['train', 'val'], loc='upper left')

# summarize history for MSE (loss)
plt.figure(2)
plt.plot(history.history['loss'][-150:])
plt.plot(history.history['val_loss'][-150:])
plt.title('MSE (loss)')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'validate'], loc='upper left')
plt.show()

