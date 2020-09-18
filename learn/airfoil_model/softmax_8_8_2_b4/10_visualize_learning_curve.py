from sklearn.model_selection import learning_curve
import numpy as np
from matplotlib import pyplot as plt
import numpy
import pandas as pd
import time
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import joblib
from keras import regularizers
from keras.layers import Activation

# https://www.kaggle.com/grfiv4/learning-curves-1
def plot_learning_curve(estimator, title, X, y, ylim=None, cv=None, scoring=None, obj_line=None,
                        n_jobs=1, train_sizes=np.linspace(.1, 1.0, 5)):

    plt.figure()
    plt.title(title)
    if ylim is not None:
        plt.ylim(*ylim)
    plt.xlabel("Training examples")
    plt.ylabel("Score")
    train_sizes, train_scores, test_scores = learning_curve(
        estimator, X, y, cv=cv, scoring=scoring, n_jobs=n_jobs, train_sizes=train_sizes)
    train_scores_mean = np.mean(train_scores, axis=1)
    train_scores_std = np.std(train_scores, axis=1)
    test_scores_mean = np.mean(test_scores, axis=1)
    test_scores_std = np.std(test_scores, axis=1)
    plt.grid()

    plt.fill_between(train_sizes, train_scores_mean - train_scores_std,
                     train_scores_mean + train_scores_std, alpha=0.1,
                     color="r")
    plt.fill_between(train_sizes, test_scores_mean - test_scores_std,
                     test_scores_mean + test_scores_std, alpha=0.1, color="g")
    plt.plot(train_sizes, train_scores_mean, 'o-', color="r",
             label="Training score")
    plt.plot(train_sizes, test_scores_mean, 'o-', color="g",
             label="Cross-validation score")

    if obj_line:
        plt.axhline(y=obj_line, color='blue')

    plt.legend(loc="best")
    return plt



from learn.airfoil_model.data_load import *

dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'

dataset = load_airfol_polar_from_dir(dir_path)

X = dataset[['Re_number', 'aoa']]
Y = dataset[['cl', 'cd']]

start_time = time.time()


def model_base():
    # create model
    model = Sequential()
    model.add(Dense(8, input_dim=2, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(8, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(2, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model


# fix random seed for reproducibility
seed = 2018

numpy.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
estimators.append(('regresor', KerasRegressor(build_fn=model_base, epochs=500, batch_size=4, verbose=1)))
pipeline = Pipeline(estimators)
# kfold = KFold(n_splits=4, random_state=seed)
# results = cross_val_score(pipeline, X, Y, cv=kfold)


# pipeline.fit(X, Y)
# print("Standardized: %.2f (%.2f) MSE" % (results.mean(), results.std()))
# your script
elapsed_time = time.time() - start_time
print("Elapsed time " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time)))

plot_learning_curve(pipeline, 'tt', X, Y).show()
