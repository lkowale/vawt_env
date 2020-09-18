import unittest
import pandas as pd
from learn.airfoil_model.data_load import *


class TestVawt(unittest.TestCase):

    def test_load_airfol_polar(self):
        # target = __import__("learn.airfoil_model.data_load")
        # self.assertEqual(target.relative_wind_num(), False)
        dir_path = '/learn/AeroDyn polars/cp10_360'
        dataset = load_airfol_polar_from_dir(dir_path, fill_aoa=True)
        test_values = [
            [20000, 10, 0.5611, 0.0918],
            [20000, -121.00, 0.7944, 1.3225],
            [100000, -175.00, 0.5238, 0.0192],
            [100000, 78.00, 0.4203, 1.7694],
            [500000, 27.00, 1.1704, 0.3691],
            [500000, 111.00, -0.7287, 1.6107]
        ]

        for test_case in test_values:
            row = dataset.loc[(dataset['Re_number'] == test_case[0]) & (dataset['aoa'] == test_case[1])]
            self.assertEqual(row['cl'].array[0], test_case[2])
            self.assertEqual(row['cd'].array[0], test_case[3])



if __name__ == '__main__':
    unittest.main()
