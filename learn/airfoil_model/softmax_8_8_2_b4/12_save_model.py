# https://machinelearningmastery.com/regression-tutorial-keras-deep-learning-library-python/
import numpy
import pandas as pd
import time
from keras.models import Sequential
from keras.layers import Dense
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.externals import joblib

start_time = time.time()

df = pd.read_csv("./VGR3d_spatial_recorder_data.csv")
# get rid of any rows that doesnt have one contour for each mask
# red_ball_side_red_cx,red_ball_side_red_cy,red_ball_side_red_quantity,red_ball_upper_red_cx,red_ball_upper_red_cy,red_ball_upper_red_quantity,x,y,z
dataset = df.dropna()
# df = df.head(20)
print(dataset['red_ball_side_red_quantity'].value_counts())
print(dataset['red_ball_upper_red_quantity'].value_counts())
# X - input has current object position and current arm position
X = dataset[['red_ball_side_red_cx','red_ball_side_red_cy','red_ball_upper_red_cx','red_ball_upper_red_cy']]
# Y - output has arm joints paramaters in both positions left - desired, right - current
Y = dataset[['x', 'y', 'z']]


# define base model
def model_base():
    # create model
    model = Sequential()
    model.add(Dense(32, input_dim=4, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(32, kernel_initializer='normal', activation='softplus'))
    model.add(Dense(3, kernel_initializer='normal', activation='linear'))
    # Compile model
    model.compile(loss='mean_squared_error', optimizer='adam', metrics=['mae'])
    return model

# fix random seed for reproducibility
seed = 2018

numpy.random.seed(seed)
estimators = []
estimators.append(('standardize', StandardScaler()))
estimators.append(('regresor', KerasRegressor(build_fn=model_base, epochs=1000, batch_size=4, verbose=1)))
pipeline = Pipeline(estimators)

pipeline.fit(X, Y)
# model_name = 'model_32x32_b4_e2200'
model_name = 'model_32x32_b4_e1000_p2'
model_file_name = model_name + '.h5'
pipeline_file_name = 'pipeline_' + model_name + '.pkl'

# Save the Keras model first:
pipeline.named_steps['regresor'].model.save(model_file_name)
# This hack allows us to save the sklearn pipeline:
pipeline.named_steps['regresor'].model = None
# Finally, save the pipeline:
# joblib.dump(pipeline, pipeline_file_name)
# used protocol=2
joblib.dump(pipeline, pipeline_file_name, protocol=2)
