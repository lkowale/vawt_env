# https://machinelearningmastery.com/regression-tutorial-keras-deep-learning-library-python/
import numpy as np
import pandas as pd
import time
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
import joblib
import os
import data_load

dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'

dataset = data_load.load_airfol_polar_from_dir(dir_path)

X = dataset[['Re_number', 'aoa']]
Y = dataset[['cl', 'cd']]

def model(layers_list):
    # create model
    model = Sequential()
    for layer in layers_list:
        model.add(layer)
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam')
    return model


# models_list = [model_32, model_64, model_128, model_256, model_32x32, model_64x64, model_128x64]
# models_list = [model_32, model_64]
# act_funct_list = ['softmax', 'elu', 'softplus', 'softsign', 'relu', 'tanh']
# models_list = []
# for afunc in act_funct_list:
#     models_list.append(model([Dense(32, input_dim=2, kernel_initializer='normal', activation=afunc),
#           Dense(32, kernel_initializer='normal', activation=afunc),
#           Dense(2, kernel_initializer='normal', activation='linear')]))

models_list = [
    model([Dense(8, input_dim=2, kernel_initializer='normal', activation='softplus'),
           Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(16, input_dim=2, kernel_initializer='normal', activation='softplus'),
           Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(32, input_dim=2, kernel_initializer='normal', activation='softplus'),
           Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(64, input_dim=2, kernel_initializer='normal', activation='softplus'),
           Dense(2, kernel_initializer='normal', activation='linear')]),
    # model([Dense(128, input_dim=2, kernel_initializer='normal', activation='softplus'),
    #        Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(8, input_dim=2, kernel_initializer='normal', activation='softplus'),
           Dense(8, kernel_initializer='normal', activation='softplus'),
           Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(16, input_dim=2, kernel_initializer='normal', activation='softplus'),
           Dense(8, kernel_initializer='normal', activation='softplus'),
           Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(32, input_dim=2, kernel_initializer='normal', activation='softplus'),
          Dense(32, kernel_initializer='normal', activation='softplus'),
          Dense(2, kernel_initializer='normal', activation='linear')]),
    model([Dense(64, input_dim=2, kernel_initializer='normal', activation='softplus'),
          Dense(32, kernel_initializer='normal', activation='softplus'),
          Dense(2, kernel_initializer='normal', activation='linear')]),
    # model([Dense(128, input_dim=2, kernel_initializer='normal', activation='softplus'),
    #       Dense(64, kernel_initializer='normal', activation='softplus'),
    #       Dense(2, kernel_initializer='normal', activation='linear')])
]

epochs_max = 100
batches_list = [4]
filename_learn = "compare_dense_learn_loss.csv"
filename_val = "compare_dense_val_loss.csv"
files = [filename_learn, filename_val]

# fix random seed for reproducibility
seed = 2018
np.random.seed(seed)
# prepare files, delete old ones if exists, add pandas columns names
for filename in files:
    if os.path.exists(filename):
        os.remove(filename)

header = ['model_name', 'elapsed_time']
for filename in files:
    with open(filename, 'w') as f:
        for item in header:
            f.write("%s," % item)
        f.write(",".join([str(x) for x in list(range(epochs_max))]))
        # f.write(','.join(map(str, list(range(epochs_max)))))
        f.write("\n")
# for filename in files:
#     with open(filename, 'w') as f:
#         for item in header:
#             f.write("%s" % item)
#             if item is not header[-1]:
#                 f.write(",")
#         f.write("\n")

x_scaler = StandardScaler()
y_scaler = StandardScaler()
X_scaled = x_scaler.fit_transform(X)
Y_scaled = y_scaler.fit_transform(Y)

for i, model in enumerate(models_list):
    for batch in batches_list:
        start_time = time.time()

        # Fit the model
        history = model.fit(X_scaled, Y_scaled, validation_split=0.25, epochs=epochs_max, batch_size=batch, verbose=1)

        # model_name = act_funct_list[i]
        model_name = ''
        for layer in model._layers[1:]:
            model_name = model_name + '_' + str(layer.units)
        model_name = model_name + '_b' + str(batch)

        elapsed_time = int(time.time() - start_time)
        # # summarize history for loss
        # plt.plot(history.history['loss'])
        # plt.plot(history.history['val_loss'])
        #     [model_name + 'loss', elapsed_time] + history.history['loss'],
        #     [model_name + 'val_loss', elapsed_time] + history.history['val_loss'],
        # ]
        model_data = [model_name, elapsed_time] + history.history['loss']
        df = pd.DataFrame([model_data]) # , columns=header + history.epochs
        with open(filename_learn, 'a') as fd:
            df.to_csv(fd, index=False, header=False, mode='a')

        model_data = [model_name, elapsed_time] + history.history['val_loss']
        df = pd.DataFrame([model_data])
        with open(filename_val, 'a') as fd:
            df.to_csv(fd, index=False, header=False, mode='a')

        print("Saved model {} with final val_loss {}".format(model_name, history.history['val_loss'][-1]))
