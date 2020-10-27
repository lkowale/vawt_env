# make a 3d plot Ct in function of blade pitch angle and blade on rotor circle position
# at given wind speed, wind direction, and rotor speed

# based on MODELING, SIMULATION, HARDWARE IMPLEMENTATION
# OF A NOVEL VARIABLE PITCH CONTROL FOR
# H–TYPE VERTICAL AXIS WIND TURBINE
# Liqun Liu — Chunxia Liu — Xuyang Zheng

import math
import vec


def tangential_force(fl, fd, a):
    """"Calculate tangential force.

    Parameters
    ----------
    fl : float
        lift force
    fd : float
        drag force
    a : int
        angle of attack

    Returns
    -------
    ft : float
        tangential force
    """
    fl_part = fl * math.sin(a)
    fd_part = fd * math.cos(a)
    return fl_part - fd_part


def normal_force(fl, fd, a):
    """"Calculate normal force.

    Parameters
    ----------
    fl : float
        lift force
    fd : float
        drag force
    a : int
        angle of attack

    Returns
    -------
    ft : float
        normal force
    """
    return fl * math.cos(a) + fd * math.sin(a)


def lift_force(r, W, c, cl):
    """"Calculate lift force. Chord length is used instead of area.

    Parameters
    ----------
    r : float
        air density in kg/m3
    W : float
        relative speed
    c : float
        chord length
    cl : float
        lift coefficient
    Returns
    -------
    ft : float
        lift force
    """
    return 0.5 * r * math.pow(W, 2) * c * cl


def drag_force(r, W, c, dl):
    """"Calculate drag force. Chord length is used instead of area.

    Parameters
    ----------
    r : float
        air density in kg/m3
    W : float
        relative speed
    c : float
        chord length
    dl : float
        drag coefficient
    Returns
    -------
    ft : float
        drag force
    """
    return 0.5 * r * math.pow(W, 2) * c * dl


# wind direction != wind vector
def relative_wind(wind_vec, blade_vec):

    rel_wind = wind_vec + blade_vec
    rel_wind = vec.Vector2(r=rel_wind.r, theta=rel_wind.theta)
    return rel_wind

    # blade_position_angle = rotor_position_angle + blade_offset
    # blade_chord_angle = blade_position_angle + math.pi / 2 + blade_pitch


def get_wind_vector(wind_dir, wind_spd):
    # wind vector has opposite direction than wind direction from where its blowing
    wind = vec.Vector2(r=wind_spd, theta=wind_dir)
    wind_oposite = wind.rotated(math.pi)
    wind_vec_angle = vec.normalize_angle(wind_oposite.theta)
    wind = vec.Vector2(r=wind_spd, theta=wind_vec_angle)
    return wind

# Fi is azimuth angle equal to base_link frame zero position
# wind direction is considered as angle in refer to base frame position
# The anticlockwise and clockwise rotation of the blade are named
# the positive rotation and the negative rotation, respectively.


def reynolds_number(fluid_velocity, airfoil_chord_length, kinematic_viscosity):
    return fluid_velocity * airfoil_chord_length / kinematic_viscosity


def tau_normalize(angle):
    if angle > math.tau:
        angle = angle % math.tau
    if angle < 0:
        angle = math.tau + angle
    return angle


def get_difference_angle(current_position, previous_position):
    if previous_position > 0 and current_position < 0:
        d_angle = (math.pi - previous_position) + (math.pi + current_position)
    else:
        d_angle = current_position - previous_position
    return d_angle


if __name__ == '__main__':
    # in degrees with respect to base frame
    wind_direction = 0
    # in m/s
    wind_speed = 3
    # in rad/s
    rotor_speed = 0

    ret = relative_wind(2, 4, 1, 1)
    print(ret)

    # print(relative_wind(2, 4, 1, 1))
    # print("f'Results of the ang: {} speed: {}'".format(relative_wind(2, 4, 1, 1)))