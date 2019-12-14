from component import *


class Gear(Component):
    """
    The gear center is at the origin and phase is the alpha from configuration
    """

    def __init__(self, configuration, radius):
        Component.__init__(self, configuration)
        self.radius = radius

    def get_global_position(self, local_joint_location):
        # return self.configuration.point + self.configuration.alignment.get_rotation_matrix() * local_joint_location
        pass


class Stick(Component):
    """
    the stick starts at the origin and it's direction is parallel to the local X axis
    """

    def __init__(self, configuration, length):
        Component.__init__(self, configuration)
        self.length = length

    def get_global_position(self, local_joint_location):
        return self.configuration.point + self.configuration.alignment.get_rotation_matrix() * local_joint_location
