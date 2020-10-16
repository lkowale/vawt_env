import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import learn.RL.rl1_qtables as rl1


class RunGame:

    def __init__(self, q_df, env):

        self.q_df = q_df
        self.env = env
        starting_theta_idx = 0
        a = self.q_df.loc[0]
        starting_pitch_idx = np.argmax(a)
        # floor division to get optimal pitch
        starting_pitch_idx = starting_pitch_idx // q_df.shape[1]
        starting_position = (starting_theta_idx, starting_pitch_idx)
        env.position = starting_position
        self.optimal_path = []
        self.optimal_path.append(starting_position)

    def run(self):

        s = self.optimal_path[0]
        total_reward = 0
        done = False
        while not done:
            a = self.q_df.columns.values[np.argmax(self.q_df.loc[s])]
            s, r, done, _ = self.env.step(a)
            total_reward += r
            self.optimal_path.append(s)
            # if theta has gone trough whole period end the game
            # if s[0] == 0:
            # last position of first index from multiindex
            if s[0] == self.q_df.index.levels[0].shape[0] - 1:
                done = True
        op_df = pd.DataFrame(self.optimal_path, columns=['theta', 'pitch'])
        return total_reward, op_df


if __name__ == '__main__':
    filename = "q_df.csv"
    # filename = '/home/aa/vawt_env/vawt/physical_model/exps/naca0018_RL/tsr3.1_q_table.csv'
    q_df = pd.read_csv(filename, index_col=[0, 1])
    q_df.columns = q_df.columns.map(int)
    airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
    # vb.VawtBlade(blade chord, airfoil_dir, rotor_radius)
    blade = vb.VawtBlade(0.2, airfoil_dir, 1)
    wind_direction = 0
    wind_speed = 3
    rotor_speed = 3
    theta_resolution = 5
    pitch_resolution = 3
    # env = VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed)
    env = rl1.VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed, theta_resolution, pitch_resolution, steps=10000)

    rg = RunGame(q_df, env)
    total_reward, plot_df = rg.run()

    plot_df.plot.scatter(x='theta', y='pitch')

    plt.show()
