import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# cross validation loss
filename = "/home/aa/vawt_env/learn/RL/eps_greedy_q_learning.csv"
df = pd.read_csv(filename, index_col=0)
# df = df[df > 50]
cov_treshold = 300
df = df.where(df > cov_treshold, 0)
# df = df.where(df <= cov_treshold, 1)
thetas = []
pitches = []
for theta in df.index.values:
    for pitch in df.columns.values:
        if df.loc[theta, pitch] > cov_treshold:
            thetas.append(theta)
            pitches.append(pitch)

plot_df = pd.DataFrame()
pitches = [float(a) for a in pitches]
plot_df['theta'] = thetas
plot_df['pitch'] = pitches


plot_df.plot.scatter(x='theta', y='pitch')

# plt.plot(thetas, pitches)

plt.show()
