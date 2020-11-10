import glob
data_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_m_5_/'
print(glob.glob("*"))
print(glob.glob(data_dir + '*'))
print(glob.glob(data_dir + "*_q_table.csv"))
# tsr_list = [f[-15:-12] for f in glob.glob(data_dir + "*_q_table.csv")]
# airfoil_name = self.blade.airfoil_dir.split('/')[-1].split('_')[0]
#tsr list
print([f.split('/')[-1].split('_')[-3] for f in glob.glob(data_dir + "*_q_table.csv")])
#wind list
print([f.split('/')[-1].split('_')[-4] for f in glob.glob(data_dir + "*_q_table.csv")])
print([f[:-12] for f in glob.glob(data_dir + "*_q_table.csv")])
