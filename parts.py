from component import *
from configuration import *


class Gear(Component):
    """
    The gear center is at the origin and phase is the alpha from configuration
    """

    def __init__(self, radius):
        Component.__init__(self)
        self.radius = radius

    def get_alignment(self):
        return self.configuration.alignment

    def get_phase_func(self, other_gear: "Gear"):
        return lambda a: -(self.radius / other_gear.radius) * a


class Actuator(Gear):

    def __init__(self):
        Gear.__init__(self, radius=0)

    def turn(self, angle):
        """
        rotate actuator a specified amount
        :param angle: number of rotations to perform. i.e. 0.5 is 180 deg, 1=360, 2=720
        """
        self.configuration.rotate_alpha(angle)


class Stick(Component):
    """
    the stick starts at the origin and it's direction is parallel to the local X axis
    """

    def __init__(self, length):
        Component.__init__(self)
        self.length = length

    def get_local_position(self):
        return self.configuration.position.vector
