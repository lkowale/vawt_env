
class VawtPhysicalModel:

    def __init__(self):

    def step(self, d_time, wind):
        # calculate blade forces
        blades_tforces = self.blades_tforces(wind)
        # calculate shaft torque
        shaft_torque = self.shaft_torque(blades_tforces)
        # calculate shaft speed
        self.shaft_speed = self.get_shaft_speed(shaft_torque)
        # update rotor position
        self.update(d_time)

        return blades_tforces, shaft_torque

    def reset(self):

    def blades_tforces(self, wind):
        blades_tforces = []
        for blade in self.blades:
            blades_tforces.append(blade.get_tforce(wind))