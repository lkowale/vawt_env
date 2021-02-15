from scipy import interpolate
import numpy as np


class InterpolateWindPower:
    def __init__(self):
        wind_speed = np.arange(0, 16)
        wind_power = [0,4,20,68,156,308,528,840,1256,1788,2452,3260,4232,5384,6724,8268]
        self.f = interpolate.interp1d(wind_speed, wind_power)

    def get_wind_power(self, speed):
        return self.f(speed)
