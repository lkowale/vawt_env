import unittest
import unittest
import pandas as pd
from learn.airfoil_model.data_load import *
import vec
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
from learn.airfoil_dynamics.ct_plot.base_calculus import *


class TestVawtBlade(unittest.TestCase):

    # def test_blade_tangential_force(self):
    #     wind_vector = vec.Vector2(r=3, theta=0)
    #     rotor_speed = 0.0001
    #     airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
    #     blade = vb.VawtBlade(0.2, airfoil_dir, 1)
    #     theta = 0
    #     pitch = 0
    #     ft = blade.get_tangential_force(wind_vector, rotor_speed, theta, pitch)
    #     # self.assertEqual(ft, 0)

    def test_blade_tangent_line_angle(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        blade = vb.VawtBlade(0.2, airfoil_dir, 1)
        # blade theta, blade tangent line angle
        test_values = [
            [-1, -2.5707963268],
            [1, -0.570796327],
            [2, 0.429203673205],
            [3, 1.42920367321],
            [-2, 2.712388980384]
        ]

        for test_case in test_values:
            answer = blade._blade_tangent_line_angle(test_case[0])
            self.assertAlmostEqual(answer, test_case[1], 7, 'Wrong blade_tangent_line_angle')

    def test_relative_wind(self):

        test_values = pd.DataFrame(
           {
               'wind_vector': [
                   vec.Vector2(r=1, theta=1),
                   vec.Vector2(r=2, theta=2),
                   vec.Vector2(r=3, theta=0.5),
                   vec.Vector2(r=4, theta=-1)
               ],
               'blade_tangent_vector': [
                   vec.Vector2(r=1, theta=-2),
                   vec.Vector2(r=1, theta=-1),
                   vec.Vector2(r=1, theta=0.1),
                   vec.Vector2(r=1, theta=3)
               ],
               'rel_wind_r': [
                    0.14, 1.02, 3.94, 3.43
               ],
               'rel_wind_theta': [
                    -0.5, 1.86, 0.4, -1.22
               ]
           }
        )

        for index, test_case in test_values.iterrows():
            answer = relative_wind(test_case['wind_vector'], test_case['blade_tangent_vector'])
            self.assertAlmostEqual(answer.r, test_case['rel_wind_r'], 2, 'Wrong relative wind')
            self.assertAlmostEqual(answer.theta, test_case['rel_wind_theta'], 2, 'Wrong relative wind')

    def test_reynolds_number(self):
        kinematic_viscosity = 1.4207E-5
        test_values = pd.DataFrame(
           {
               'wind_speed': [
                    3, 6, 9
               ],
               'chord_length': [
                    0.4, 0.35, 0.3
               ],
               're_number': [
                   84465, 147814, 190047
               ]
           }
        )
        for index, test_case in test_values.iterrows():
            answer = reynolds_number(test_case['wind_speed'], test_case['chord_length'], kinematic_viscosity)
            self.assertAlmostEqual(answer, test_case['re_number'], 0, 'Wrong reynolds number')

    def test_blade_chord_vec(self):
        # calculate blade chord angle on blade_tangent_angle and pitch
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        blade = vb.VawtBlade(0.2, airfoil_dir, 1)
        test_values = [
               # pitch positive - rotates clockwise
               # positive pitch - 'takes wind under the wing of ascending plane'
               # positive pitch must be substracted from blade tangent to give actual blade chord vector
               # blade_tangent_vector, pitch, blade_chord_vector.theta
               [vec.Vector2(r=1, theta=-2), 1, -1],
               [vec.Vector2(r=1, theta=-1), -0.5, -1.5],
               [vec.Vector2(r=1, theta=0.1), 1, 1.1],
               [vec.Vector2(r=1, theta=3), -1, 2]
           ]
        for tc in test_values:
            answer = blade._blade_chord_vec(tc[0], tc[1])
            self.assertAlmostEqual(answer.theta, tc[2], 5, 'Wrong blade chord vector')

    def test_angle_of_attack(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        blade = vb.VawtBlade(0.2, airfoil_dir, 1)
        test_values = [
               # rel_wind_vector, blade_chord_vector, aoa_rad, aoa_360
               [vec.Vector2(r=1, theta=-2.356194490192345), vec.Vector2(r=1, theta=-1.5707963267948966), -0.785398163397, -45],
               [vec.Vector2(r=1, theta=-1), vec.Vector2(r=1, theta=-0.0007162025873375634), -0.9992837974126625, -57.25474412755153],
               [vec.Vector2(r=1, theta=0.1), vec.Vector2(r=1, theta=-2), 2.1, 120.3211369],
               [vec.Vector2(r=1, theta=3), vec.Vector2(r=1, theta=-2), -1.28318530,	-73.5211024]
           ]
        for tc in test_values:
            aoa_rad, aoa_360 = blade._angle_of_attack(tc[0], tc[1])
            self.assertAlmostEqual(aoa_rad, tc[2], 5, 'Wrong aoa_rad')
            self.assertAlmostEqual(aoa_360, tc[3], 5, 'Wrong aoa_360')

    def test_get_cl_cd_coeffs(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/cp10_360'
        blade = vb.VawtBlade(0.2, airfoil_dir, 1)
        test_values = [
               # aoa_360, re_number, cl, cd
               [-78.00, 20000, -0.3640, 1.7268],
               [26.00, 50000, 0.5911, 0.3391],
               [176.00, 200000, -0.3291, 0.0145],
               [-149.00, 1000000, 0.3591, 0.4565]
           ]
        for tc in test_values:
            cl, cd = blade.get_coeffs(tc[0], tc[1])
            self.assertAlmostEqual(cl, tc[2], 5, 'Wrong cl coeff')
            self.assertAlmostEqual(cd, tc[3], 5, 'Wrong cd coeff')

    def test_get_wind_vector(self):
        test_values = [
            [-1, 0.5, vec.Vector2(r=0.5, theta=2.14159265359)],
            [1, 2.5, vec.Vector2(r=2.5, theta=-2.14159265359)],
            [2, 3.5, vec.Vector2(r=3.5, theta=-1.14159265359)],
            [3, 4.5, vec.Vector2(r=4.5, theta=-0.14159265359)],
            [-2, 2, vec.Vector2(r=2, theta=1.14159265359)]
        ]

        for tc in test_values:
            wind_direction = tc[0]
            wind_speed = tc[1]
            answer = get_wind_vector(wind_direction, wind_speed)
            self.assertAlmostEqual(answer.theta, tc[2].theta, 5, 'Wrong get_wind_vector theta')
            self.assertAlmostEqual(answer.r, tc[2].r, 5, 'Wrong get_wind_vector radius')

    def test__rel_tang_angle(self):
        airfoil_dir = '/home/aa/vawt_env/learn/AeroDyn polars/naca0018_360'
        blade = vb.VawtBlade(0.2, airfoil_dir, 1)
        # VECTORS rel_wind, blade_tangent_vector
        test_values = [
            [vec.Vector2(r=1, theta=1), vec.Vector2(r=1, theta=1.2), -0.2],
            [vec.Vector2(r=1, theta=1), vec.Vector2(r=1, theta=1.2), -0.2]
        ]

        for rel_wind, blade_tangent, rel_tang_angle in test_values:
            self.assertAlmostEqual(blade._rel_tang_angle(rel_wind, blade_tangent), rel_tang_angle, 5, 'Wrong _rel_tang_angle')



if __name__ == '__main__':
    unittest.main()
