y = 0.95
eps = 0.5
lr = 0.8
decay_factor = 0.999
num_episodes = 500
for i in range(num_episodes):
    eps *= decay_factor
print(eps)
