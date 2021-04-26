import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
import unittest
from .physical_model import RotorBlade
import numpy as np

airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_10_m/'


class TestVawtOp(unittest.TestCase):

    def test_get_op(self):

        # joint_name, chord_length, height, offset, sa_radius, airfoil_dir, op_interpolator_dir
        blade_1 = RotorBlade('blade_1_joint', 0.2, 2, 0, 1, airfoil_dir, op_interp_dir)
        # blade, wind_direction, wind_speed, tsr, theta, op
        data = [
            (blade_1, 0, 3, 1, 0, 18.07),
            (blade_1, 0, 3, 1, 1, 18.07),
            (blade_1, 0, 3, 1, -1, 18.07),
            (blade_1, 0, 7, 1, 1, 18.07),
            (blade_1, 0, 3.5, 1, 0, 0),
            (blade_1, 0, 3.5, 2.5, 0, 0),
            (blade_1, 0, 3.7, 2.7, -1.5, 0)
            # (blade_1, 0, 8, 40, -1, 75.8169500791),
            # (blade_1, 0, 8, 40, -3, 75.8169500791),
            # (blade_1, 0, 3, 3, 0, 2.44937957791787),
            # (blade_1, np.pi, 3, 3, -np.pi, 2.44937957791787),
            # (blade_1, 0, 6, 12, 0, 13.297044148331983),
            # (blade_2, np.pi, 3, 3, 0, 2.44937957791787),
            # (blade_2, 0, 3, 3, -np.pi, 2.44937957791787),
            # (blade_2, 0, 6, 12, -np.pi, 13.297044148331983)
        ]

        for blade, wind_direction, wind_speed, tsr, theta, op in data:
            wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
            # TSR = ωR/V∞ -> w=TSR*wind_speed/R
            # tsr = rotor_speed * blade.sa_radius / wind_speed
            # set pitch #wind_speed, tsr, rotor_theta, wind_direction
            calculated = blade.get_optimal_pitch(wind_speed, tsr, theta, wind_direction)

            self.assertAlmostEqual(calculated, op, 1, 'Wrong Ft calculation')

if __name__ == '__main__':
    unittest.main()
