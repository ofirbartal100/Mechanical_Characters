import numpy as np
from scipy.spatial.transform import Rotation


class Point:
    """
    represent a 3d point in space
    """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def vector(self):
        return np.array([self.x, self.y, self.z])


class Alignment:
    """
    represent a global 3d euler angles in degrees alignment
    the angles correspond to rotation axes in the following way:
    gamma: x
    beta: y
    alpha: z
    """

    def __init__(self, gamma, beta, alpha):
        self.gamma = gamma
        self.beta = beta
        self.alpha = alpha

    def vector(self):
        return np.array([self.gamma, self.beta, self.alpha])

    def get_rotation_obj(self):
        return Rotation.from_euler('xyz', self.vector(), degrees=True)


class Configuration:
    def __init__(self, position, alignment):
        """
        represents the configuration of a component.
        contains global position T=(x,y,z)
        contains global rotation Euler=(gamma,beta,alpha)
        """

        self.position = position
        self.alignment = alignment

    def rotate_alpha(self, d_alpha):
        self.alignment.alpha += d_alpha

    def rotate_beta(self, d_beta):
        self.alignment.alpha += d_beta

    def rotate_gamma(self, d_gamma):
        self.alignment.alpha += d_gamma

    def move_x(self, d_x):
        self.position.x += d_x

    def move_y(self, d_y):
        self.position.y += d_y

    def move_z(self, d_z):
        self.position.z += d_z
