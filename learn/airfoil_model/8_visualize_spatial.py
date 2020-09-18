import pandas as pd
import matplotlib.pyplot as plt

filename = "cmp_act_val_loss.csv"

df = pd.read_csv(filename, index_col='model_name')
# drop time column
df.drop('elapsed_time', axis=1, inplace=True)
df = df.T
df.index = df.index.astype('int64')
val_loss = df

filename = "cmp_act_learn_loss.csv"

df = pd.read_csv(filename, index_col='model_name')
# drop time column
df.drop('elapsed_time', axis=1, inplace=True)
df = df.T
df.index = df.index.astype('int64')
learn_loss = df

rows = 3
columns = 2
fig_loss, axs = plt.subplots(rows, columns)

val_loss.plot(ax=axs[0, 0], title='val_loss')
learn_loss.plot(ax=axs[0, 1], title='learn_loss')

for i, model_name in enumerate(val_loss.columns):
    learn_val = pd.DataFrame()
    learn_val['learn'] = learn_loss[model_name]
    learn_val['validate'] = val_loss[model_name]
    learn_val.plot(ax=axs[i//2, i % 2], title=model_name)

fig_mae, axs_mae = plt.subplots(rows, columns)



# axs[0, 0].plot(x, y)
# axs[0, 0].set_title('Axis [0,0]')
# axs[0, 1].plot(x, y, 'tab:orange')
# axs[0, 1].set_title('Axis [0,1]')
# axs[1, 0].plot(x, -y, 'tab:green')
# axs[1, 0].set_title('Axis [1,0]')
# axs[1, 1].plot(x, -y, 'tab:red')
# axs[1, 1].set_title('Axis [1,1]')

plt.show()