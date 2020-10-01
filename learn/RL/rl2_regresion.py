import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# cross validation loss
filename = "/home/aa/vawt_env/learn/RL/eps_greedy_q_learning.csv"
df = pd.read_csv(filename, index_col=0)
# df = df[df > 50]
cov_treshold = 300
# df = df.where(df > cov_treshold, 0)
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

fig = plt.figure()
ax = fig.add_subplot(311)
ax.scatter(thetas, pitches)
# plot_df.plot.scatter(x='theta', y='pitch')

# plt.plot(thetas, pitches)



from symfit import parameters, variables, sin, cos, Fit
import numpy as np
import matplotlib.pyplot as plt

def fourier_series(x, f, n=0):
    """
    Returns a symbolic fourier series of order `n`.

    :param n: Order of the fourier series.
    :param x: Independent variable
    :param f: Frequency of the fourier series
    """
    # Make the parameter objects for all the terms
    a0, *cos_a = parameters(','.join(['a{}'.format(i) for i in range(0, n + 1)]))
    sin_b = parameters(','.join(['b{}'.format(i) for i in range(1, n + 1)]))
    # Construct the series
    series = a0 + sum(ai * cos(i * f * x) + bi * sin(i * f * x)
                     for i, (ai, bi) in enumerate(zip(cos_a, sin_b), start=1))
    return series

x, y = variables('x, y')
w, = parameters('w')
model_dict = {y: fourier_series(x, f=w, n=3)}
print(model_dict)

# Make step function data
xdata = np.array(thetas)
ydata = np.array(pitches)
# Define a Fit object for this model and data
fit = Fit(model_dict, x=xdata, y=ydata)
fit_result = fit.execute()
print(fit_result)

# Plot the result
# plt.plot(xdata, ydata)
f_model = fit.model(x=xdata, **fit_result.params)
# plt.plot(xdata, f_model.y, ls=':')
# ax = fig.add_subplot(312)
ax.plot(xdata, f_model.y)


xdata = np.linspace(-np.pi, 4*np.pi)
f_model = fit.model(x=xdata, **fit_result.params)
# plt.plot(xdata, f_model.y, ls=':')
ax = fig.add_subplot(313)
ax.plot(xdata, f_model.y)

plt.show()
