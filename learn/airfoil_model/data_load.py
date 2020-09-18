import numpy as np
from os.path import isfile, join
from os import listdir
import pandas as pd
import math
from scipy import interpolate


# load cl,cd  from areodyn format files
def load_airfol_polar_from_dir(directory_path, fill_aoa=False):
    files_list = [f for f in listdir(directory_path) if isfile(join(directory_path, f))]
    # print(files_list)

    coll_data = []
    # load each files as pandas DataFrame
    # nedds to be xfoil type file
    for file in files_list:
        # open file
        fh = open(directory_path + '/' + file, 'r')
        lines = fh.readlines()
        # get Reynolds number line
        line = lines[1]
        words = line.split()
        words = words[1].split('_')
        Re_number = words[1][2:]
        Re_number = float(Re_number) * math.pow(10, 6)
        data = [line.split() for line in lines[14:-2]]
        # data = [line.append(Re_number) for line in data]
        for line in data:
            line = line.append(Re_number)
            # change columns datatypes
        coll_data = coll_data + data
        # close
        fh.close()
    columns_names = ['aoa', 'cl', 'cd', 'Re_number']
    type_dir = {column_name: float for column_name in columns_names}
    dataset = pd.DataFrame(coll_data, columns=columns_names)
    dataset = dataset.astype(type_dir)
    if fill_aoa:
        dataset = _fill_aoa(dataset)
    return dataset


def _fill_aoa(dataset):
    unique, counts = np.unique(dataset['Re_number'], return_counts=True)
    coll_df = pd.DataFrame()
    for re_number in unique:
        re_group = dataset.loc[dataset['Re_number'] == re_number][['aoa', 'cl', 'cd']]
        x = re_group['aoa']
        y_cl = re_group['cl']
        y_cd = re_group['cd']
        f_cl = interpolate.interp1d(x, y_cl)
        f_cd = interpolate.interp1d(x, y_cd)
        aoa_range = np.arange(-180, 180, 1)
        new_y_cl = f_cl(aoa_range)
        new_y_cd = f_cd(aoa_range)
        re_df = pd.DataFrame()
        re_df['aoa'] = aoa_range
        re_df['cl'] = new_y_cl
        re_df['cd'] = new_y_cd
        re_df['Re_number'] = re_number
        if coll_df.empty:
            coll_df = re_df
        else:
            coll_df = coll_df.append(re_df)
    return coll_df

if __name__ == "__main__":
    dir_path = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
    dataset = load_airfol_polar_from_dir(dir_path, fill_aoa=True)
