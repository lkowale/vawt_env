
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

def find_optimum_params(self):
    # for given blade get  params for fourier series and sve them as csv
    # get list of optimal pitch fourier params
    fourier_params = []
    for tsr in np.arange(0.1, 5.0, 0.3):
        # fourier_params.append([tsr, *self.get_fourier_params(tsr)])
        fourier_params.append(
            collections.OrderedDict([('tsr', tsr)]).update(self.save_q_table_tsr(tsr, self.coverage_treshold)))
    # get dataframe of optimal pitch fourier params
    # fourier_params_df = pd.DataFrame(fourier_params, index_col=0)
    # https://stackoverflow.com/questions/44365209/generate-a-pandas-dataframe-from-ordereddict
    fourier_params_df = pd.DataFrame(fourier_params)
    # save dataframe to csv
    file_name = blade.airfoil_dir.split('/')[-1] + "_fourier_params.csv"
    fourier_params_df.to_csv(file_name)


def get_fourier_params(self, tsr, coverage_treshold):
    # set folder and files base name
    airfoil_name = self.blade.airfoil_dir.split('/')[-1].split('_')[0]
    folder_name = airfoil_name + '_RLcover'
    file_base_name = "/tsr{:1.1f}".format(tsr)
    base_file_path = folder_name + file_base_name

    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    # set environment
    wind_direction = 0
    wind_speed = 1
    rotor_speed = wind_speed * tsr
    theta_resolution = 5
    pitch_resolution = 3
    rl_environment = rl1.VawtRLEnvironment(blade, wind_direction, wind_speed, rotor_speed, theta_resolution,
                                           pitch_resolution)
    # get RL occupation/coverage dataframe and save plot of coverage + tangent coeef
    file_name = base_file_path + ".png"
    q_df, coverage_df = rl1.eps_greedy_q_learning_with_table(rl_environment, 20, save_file_name=file_name)
    # save the coverage dataframe
    file_name = base_file_path + ".csv"
    coverage_df.to_csv(file_name)
    # save q_table
    file_name = base_file_path + "_q_table.csv"
    q_df.to_csv(file_name)
    print("Saved coverage dataframe to " + file_name)

    # # use treshold on coverage dataframe to pull up only mostly used actions/pitches
    # thetas = []
    # pitches = []
    # for theta in coverage_df.index.values:
    #     for pitch in coverage_df.columns.values:
    #         if coverage_df.loc[theta, pitch] > coverage_treshold:
    #             thetas.append(theta)
    #             pitches.append(pitch)
    # run the game once to get optimal path and total reward
    rg = rl3.RunGame(q_df, rl_environment)
    total_reward, plot_df = rg.run()

    # make fourier series regression
    x, y = variables('x, y')
    w, = parameters('w')
    model_dict = {y: fourier_series(x, f=w, n=3)}
    xdata = np.array(thetas)
    ydata = np.array(pitches)
    fit = Fit(model_dict, x=xdata, y=ydata)
    fit_result = fit.execute()
    return fit_result.params