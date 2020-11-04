import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
import unittest
from .physical_model import RotorBlade
import numpy as np


class TestVawtPm(unittest.TestCase):

    def test_blade_tangent_line_angle(self):
        wind_direction = 0
        wind_speed = 3

        wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
        # rotor_speed = 0.0001
        rotor_speed = 3
        # airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'

        blade = vb.VawtBlade(0.2, airfoil_dir, 1)

        theta_pitch = [
            (-2, 0, 0.252998783),
            (-1, 0, -0.1313296584),
            (-1, 20 * math.tau / 360, -1.4047434233043994),
            (-2, -2, -5.047621777237812)
        ]

        for theta, pitch, ft in theta_pitch:
            self.assertAlmostEqual(blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch), ft,
                                   7, 'tangent_line_angle')

    def test_get_tforce(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
        op_interp_dir = '/home/aa/vawt_env/vawt/physical_model/pitch_optimizer/exps/naca0018_RL_4/'
        # joint_name, chord_length, height, offset, sa_radius, airfoil_dir, op_interpolator_dir
        blade_1 = RotorBlade('blade_1_joint', 0.2, 2, 0, 1, airfoil_dir, op_interp_dir)
        blade_2 = RotorBlade('blade_2_joint', 0.2, 2, np.pi, 1, airfoil_dir, op_interp_dir)
        # blade, wind_direction, wind_speed, rotor_speed, theta, tf
        data = [
            (blade_1, 0, 8, 40, -1, 75.8169500791),
            (blade_1, 0, 8, 40, -3, 75.8169500791),
            (blade_1, 0, 3, 3, 0, 2.44937957791787),
            (blade_1, np.pi, 3, 3, np.pi, 2.44937957791787),
            (blade_1, 0, 6, 12, 0, 13.297044148331983),
            (blade_2, np.pi, 3, 3, 0, 2.44937957791787),
            (blade_2, 0, 3, 3, np.pi, 2.44937957791787),
            (blade_2, 0, 6, 12, np.pi, 13.297044148331983)
        ]

        for blade, wind_direction, wind_speed, rotor_speed, theta, tf in data:
            wind_vector = bc.get_wind_vector(wind_direction, wind_speed)
            # TSR = ωR/V∞ -> w=TSR*wind_speed/R
            tsr = rotor_speed * blade.sa_radius / wind_speed
            # set pitch
            blade.pitch = blade.get_optimal_pitch(tsr, theta, wind_direction)
            calculated = blade.get_tforce(wind_vector, rotor_speed, theta)
            self.assertAlmostEqual(calculated, tf, 7, 'Wrong Ft calculation')

if __name__ == '__main__':
    unittest.main()
