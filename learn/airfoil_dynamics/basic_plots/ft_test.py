import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
import math
import learn.airfoil_dynamics.ct_plot.base_calculus as bc
import unittest


class TestVawtFt(unittest.TestCase):

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
            (-1, 0, -0.00022723),
            (-1, 20 * math.tau / 360),
            (-2, -2)
        ]

        for theta, pitch, ft in theta_pitch:
            self.assertAlmostEqual(blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch), ft,
                                   7, 'Wrong Ft calculation')


if __name__ == '__main__':
    unittest.main()
