#!/usr/bin/env python
import numpy as np
import math
import vawt.physical_model.physical_model as pm
import pandas as pd
import matplotlib.pyplot as plt
import learn.airfoil_dynamics.ct_plot.vawt_blade as vb
from vawt.physical_model.pitch_optimizer.po5_rotor_tr_2b import VawtTest
from matplotlib import cm
from vawt.physical_model.wind_power_interpolation import InterpolateWindPower
import vawt.physical_model.power_comparision_parameters as pam
import unittest

from vawt.physical_model.work_power import VawtPowerTest


class TestVawtWp(unittest.TestCase):

    def test_get_tforce(self):
        _blades = [
            # blades_joint_name, chord_length, height, offset, sa_radius, airfoil_dir, optimal_path_dir
            pm.RotorBlade('blade_1_joint', 0.2, 2, 0, pam.params['blade_shaft_dist'], pam.params['airfoil_dir'],
                          pam.params['op_interp_dir']),
            pm.RotorBlade('blade_2_joint', 0.2, 2, 2 * np.pi / 3, pam.params['blade_shaft_dist'],
                          pam.params['airfoil_dir'], pam.params['op_interp_dir']),
            pm.RotorBlade('blade_3_joint', 0.2, 2, -2 * np.pi / 3, pam.params['blade_shaft_dist'],
                          pam.params['airfoil_dir'], pam.params['op_interp_dir'])
        ]
        vpm_tb = pm.VawtPhysicalModel(_blades)

        # wind_direction, wind_speed, tsr, work
        data = [
            (0, 5, 2, 163.06),
            (0, 3, 1, 20.11)
        ]

        for wind_direction, wind_speed, tsr, work in data:

            params = {
                'wind_speed': wind_speed,
                'wind_direction': wind_direction
            }
            # tsr = self.parameters['rotor_speed'] * self.rot_blade.sa_radius / self.parameters['wind_speed']
            params["rotor_speed"] = tsr * wind_speed / 1  # tempoarly set static blade radius
            # gen plots of torque in function of rotor theta to check if there are any negative positions
            vt = VawtTest(vpm_tb, params, 100)
            calculated = vt.work_per_revolution()
            self.assertAlmostEqual(calculated, work, 1, 'Wrong Work calculation')

if __name__ == '__main__':
    unittest.main()
