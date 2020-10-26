import glob
data_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_1/'
print(glob.glob(data_dir + '*'))
print(glob.glob("*"))
# tsr_list = [f[-15:-12] for f in glob.glob(data_dir + "*_q_table.csv")]