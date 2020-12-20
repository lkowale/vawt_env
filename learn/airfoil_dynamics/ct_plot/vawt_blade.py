import pandas as pd
import math
from learn.airfoil_model.data_load import *
from learn.airfoil_dynamics.ct_plot.base_calculus import *
import vec

air_density = 1.225
kinematic_viscosity = 1.733e-5


class VawtBlade:

    def __init__(self, chord_length, airfoil_dir, rotor_radius):
        self.rotor_radius = rotor_radius
        self.chord_length = chord_length
        self.airfoil_dir = airfoil_dir
        self.cl_cd_df = load_airfol_polar_from_dir(self.airfoil_dir, fill_aoa=True)
        self.cl_interpolate = self.get_cl_interpolate()
        self.cd_interpolate = self.get_cd_interpolate()

    def get_cl_interpolate(self):
        df = self.cl_cd_df
        x = df['aoa'].unique()
        y = df['Re_number'].unique()
        z = df['cl'].values.reshape(y.size, x.size)
        return interpolate.interp2d(x, y, z, kind='cubic')

    def get_cd_interpolate(self):
        df = self.cl_cd_df
        x = df['aoa'].unique()
        y = df['Re_number'].unique()
        z = df['cd'].values.reshape(y.size, x.size)
        return interpolate.interp2d(x, y, z, kind='cubic')

    def get_tangential_force(self, wind_vector, rotor_speed, theta, pitch):
        """"Calculate tangential force. Chord length is used instead of area.

        Parameters
        ----------
        wind_vector : vec
            vector describes wind speed and direction
        rotor_speed : float
            rotor angular speed in rad/s
        theta : float
            position of blade relative to rotor 0 position in rad
        pitch : float
            angle between blade chord line and tangent line in rad
        Returns
        -------
        ft : float
            tangential force
        """
        # relative wind vector
        try:
            blade_speed = rotor_speed * self.rotor_radius
        except TypeError:
            pass
        blade_tangent_line_angle = self._blade_tangent_line_angle(theta)

        blade_tangent_vector = vec.Vector2(r=blade_speed, theta=blade_tangent_line_angle)
        rel_wind = relative_wind(wind_vector, blade_tangent_vector)

        re_number = reynolds_number(rel_wind.r, self.chord_length, kinematic_viscosity)
        blade_chord_vector = self._blade_chord_vec(blade_tangent_vector, pitch)
        aoa_rad, aoa_360 = self._angle_of_attack(rel_wind, blade_chord_vector)
        cl, cd = self.get_coeffs(aoa_360, re_number)
        fl = lift_force(air_density, rel_wind.r, self.chord_length, cl)
        fd = drag_force(air_density, rel_wind.r, self.chord_length, cd)
        # casting angle is between rel_wind and blade tangent
        casting_angle = self._rel_tang_angle(rel_wind, blade_tangent_vector)
        return tangential_force(fl, fd, casting_angle)

    def get_tangential_force_talker(self, wind_vector, rotor_speed, theta, pitch):
        """"Calculate tangential force. Chord length is used instead of area.

        Parameters
        ----------
        wind_vector : vec
            vector describes wind speed and direction
        rotor_speed : float
            rotor angular speed in rad/s
        theta : float
            position of blade relative to rotor 0 position in rad
        pitch : float
            angle between blade chord line and tangent line in rad
        Returns
        -------
        ft : float
            tangential force
        """
        # relative wind vector
        blade_speed = rotor_speed * self.rotor_radius
        blade_tangent_line_angle = self._blade_tangent_line_angle(theta)

        blade_tangent_vector = vec.Vector2(r=blade_speed, theta=blade_tangent_line_angle)
        rel_wind = relative_wind(wind_vector, blade_tangent_vector)

        re_number = reynolds_number(rel_wind.r, self.chord_length, kinematic_viscosity)
        blade_chord_vector = self._blade_chord_vec(blade_tangent_vector, pitch)
        aoa_rad, aoa_360 = self._angle_of_attack(rel_wind, blade_chord_vector)
        cl, cd = self.get_coeffs(aoa_360, re_number)
        fl = lift_force(air_density, rel_wind.r, self.chord_length, cl)
        fd = drag_force(air_density, rel_wind.r, self.chord_length, cd)
        return tangential_force(fl, fd, aoa_rad), fl, fd, cl, cd, aoa_rad, aoa_360, blade_chord_vector, re_number, rel_wind, blade_tangent_vector

    def get_visualization_vectors(self, wind_vector, rotor_speed, theta, pitch):

        vectors = []
        # relative wind vector
        blade_speed = rotor_speed * self.rotor_radius
        blade_tangent_line_angle = self._blade_tangent_line_angle(theta)

        blade_tangent_vector = vec.Vector2(r=blade_speed, theta=blade_tangent_line_angle)
        vectors.append(('blade_tangent', blade_tangent_vector))
        rel_wind = relative_wind(wind_vector, blade_tangent_vector)
        vectors.append(('relative_wind', relative_wind))
        re_number = reynolds_number(rel_wind.r, self.chord_length, kinematic_viscosity)
        blade_chord_vector = self._blade_chord_vec(blade_tangent_vector, pitch)
        vectors.append(('blade_chord', blade_chord_vector))
        aoa_rad, aoa_360 = self._angle_of_attack(rel_wind, blade_chord_vector)
        cl, cd = self.get_coeffs(aoa_360, re_number)

        fl = lift_force(air_density, rel_wind.r, self.chord_length, cl)
        # lift force vector is perpendicular to rel_wind vector
        fl_vec = vec.Vector2(r=fl, theta=rel_wind.rotated(math.pi/2).theta)
        vectors.append(('fl_vec', fl_vec))

        fd = drag_force(air_density, rel_wind.r, self.chord_length, cd)
        # drag force is opposite to rel_wind
        fd_vec = vec.Vector2(r=fd, theta=rel_wind.rotated(math.pi).theta)
        vectors.append(('fd_vec', fd_vec))

        tf = tangential_force(fl, fd, aoa_rad)
        # tangential force vector is opposite to blade tangential speed vector
        tangential_force_vec = vec.Vector2(r=tf, theta=blade_tangent_vector.rotated(math.pi).theta)
        vectors.append(('tangential_force_vec', tangential_force_vec))
        return vectors

    def get_coeffs(self, aoa, re_number):
        cl = self.cl_interpolate(aoa, re_number)[0]
        cd = self.cd_interpolate(aoa, re_number)[0]
        return cl, cd

    def _blade_tangent_line_angle(self, theta):
        btla = theta - math.pi / 2
        btla = vec.normalize_angle(btla)
        return btla

    def _blade_chord_vec(self, blade_tangent_vector, pitch):
        """"Calculate blade chord vector.=
               pitch positive - rotates clockwise
               positive pitch - 'takes wind under the wing of ascending plane'
               positive pitch must be substracted from blade tangent to give actual blade chord vector
        Parameters
        ----------
        blade_tangent_vector : vec
            vector describes a line tangent to rotor in blade position
        pitch : float
            angle between blade chord line and blade_tangent_vector in rad
        Returns
        -------
        blade_chord_vec : float
            vector describes blade chord absolute angle and its linear speed

        """
        # blade_chord_vec = blade_tangent_vector.theta - pitch
        # return tau_normalize(blade_chord_vec)
        return blade_tangent_vector.rotated(pitch)

    def _angle_of_attack(self, rel_wind, blade_chord):
        """"Calculate angle of attack

        Parameters
        ----------
        relative_wind : vec
        blade_chord : vec

        Returns
        -------
        aoa_rad : float
            angle of attatck in radians
        aoa_360 : float
            angle of attatck in degrees
        """
        # blade_chord_vector - (relative_wind + pi)
        # rel_oposite = rel_wind.rotated(math.pi)
        aoa_rad = rel_wind.theta - blade_chord.theta
        aoa_rad = vec.normalize_angle(aoa_rad)
        aoa_360 = aoa_rad * 360 / math.tau
        return aoa_rad, aoa_360

    def _rel_tang_angle(self, rel_wind, blade_tangent):
        return rel_wind.theta - blade_tangent.theta
        # return blade_tangent.theta - rel_wind.theta

