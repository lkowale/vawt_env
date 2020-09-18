import numpy as np
import pandas as pd
import time
from keras.models import Sequential
from keras.layers import Dense, Dropout
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from keras import regularizers
from keras.layers import Activation
import joblib
import os
from learn.airfoil_model.data_load import *

dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'

dataset = load_airfol_polar_from_dir(dir_path)

X = dataset[['Re_number', 'aoa']]
Y = dataset[['cl', 'cd']]


def model_base():
    # create model
    model = Sequential()
    model.add(Dense(8, input_dim=2, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(8, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(2, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model

seed = 2019
np.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
epochs = 100
batch_size = 4
estimators.append(('regresor', KerasRegressor(build_fn=model_base, epochs=epochs, batch_size=batch_size, verbose=1)))
pipeline = Pipeline(estimators)
kfold = KFold(n_splits=4, shuffle=True, random_state=seed)

start_time = time.time()
results = cross_val_score(pipeline, X, Y, cv=kfold)
elapsed_time = time.time() - start_time

with open(os.path.basename(__file__), 'a') as fd:
    model = model_base()
    model_name = "model"
    for layer in model._layers[1:]:
        if layer.name.startswith('dense'):
            app = str(layer.units)
        if layer.name.startswith('activ'):
            app = 'actv'
        if layer.name.startswith('drop'):
            app = 'dp' + str(layer.rate)

        model_name = model_name + '_' + app
        
    model_name = model_name + '_b' + str(batch_size)
    model_name = model_name + ' Epochs:' + str(epochs)
    message = "\n# " + model_name + " Std: {:.2f} ({:.2f}) MSE  Elapsed time:{}".format(results.mean(), results.std(), time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

    fd.write(message)

print(results)

# pipeline.fit(X, Y)
print(" Std: %.2f (%.2f) MSE" % (results.mean(), results.std()))
# your script

print("Elapsed time " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))
# model_8_8_2_b4 Epochs:500 Std: -0.20 (0.02) MSE  Elapsed time:00:01:52
# model_8_8_2_b4 Epochs:100 Std: -0.20 (0.02) MSE  Elapsed time:00:00:24