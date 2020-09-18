# https://machinelearningmastery.com/regression-tutorial-keras-deep-learning-library-python/
import numpy as np
from os.path import isfile, join
from os import listdir
import pandas as pd
import time
import math
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from learn.airfoil_model.data_load import load_airfol_polar_from_dir

# from sklearn.externals import joblib
start_time = time.time()
# make a list of all files in given directory
dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'

dataset = load_airfol_polar_from_dir(dir_path)

X = dataset[['Re_number', 'aoa']]
Y = dataset[['cl']]


# define base model
def baseline_model():
    # create model
    model = Sequential()
    model.add(Dense(8, input_dim=2, kernel_initializer='normal', activation='relu'))
    model.add(Dense(8, kernel_initializer='normal', activation='relu'))
    model.add(Dense(1, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model

# fix random seed for reproducibility
seed = 2018

np.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
estimators.append(('regresor', KerasRegressor(build_fn=baseline_model, epochs=150, batch_size=4, verbose=1)))
pipeline = Pipeline(estimators)
kfold = KFold(n_splits=4, random_state=seed)
results = cross_val_score(pipeline, X, Y, cv=kfold)


# pipeline.fit(X, Y)
print("Standardized: %.2f (%.2f) MSE" % (results.mean(), results.std()))
# your script
elapsed_time = time.time() - start_time
print("Elapsed time " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

# # print("Kfold results")
# # print(results)
# model_name = "model"
# for layer in pipeline.named_steps['regresor'].model._layers[1:]:
#     model_name = model_name + '_' + str(layer.units)
#
# # Save the Keras model first:
# pipeline.named_steps['regresor'].model.save(model_name + '.h5')
# # This hack allows us to save the sklearn pipeline:
# pipeline.named_steps['regresor'].model = None
# # Finally, save the pipeline:
# joblib.dump(pipeline, 'pipeline_' + model_name + '.pkl')
