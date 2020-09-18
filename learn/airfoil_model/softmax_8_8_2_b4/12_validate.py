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
from keras.models import load_model
import matplotlib.pyplot as plt
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
X = dataset[['red_ball_side_red_cx', 'red_ball_side_red_cy', 'red_ball_upper_red_cx', 'red_ball_upper_red_cy']]
# Y - output has arm joints paramaters in both positions left - desired, right - current
Y = dataset[['x', 'y', 'z']]

# fix random seed for reproducibility
seed = 2018

np.random.seed(seed)


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


model_name = 'model_32x32_b4_e2200'
model_file_name = model_name + '.h5'
pipeline_file_name = 'pipeline_' + model_name + '.pkl'

# Load the pipeline first:
pipeline = joblib.load(pipeline_file_name)

# Then, load the Keras model:
pipeline.named_steps['regresor'].model = load_model(model_file_name)

Y_pred = pipeline.predict(X)

diff = Y - Y_pred
diff = diff.astype(int)

elbow = diff.iloc[:, 0].value_counts().to_frame()
shoulder = diff.iloc[:, 1].value_counts().to_frame()
torso = diff.iloc[:, 2].value_counts().to_frame()

print(elbow)
print(shoulder)
print(torso)
