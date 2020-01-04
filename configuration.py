class Point():
    """
    represent a 3d point in space
    """

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Alignment():
    """
    represent a global 3d euler angles alignment
    """

    def __init__(self, gamma, beta, alpha):
        self.gamma = gamma
        self.beta = beta
        self.alpha = alpha


class Configuration():
    def __init__(self, point, alignment):
        """
        represents the configuration of a component.
        contains global position T=(x,y,z)
        contains global rotation Euler=(gamma,beta,alpha)
        """

        self.point = point
        self.alignment = alignment

    def rotate_alpha(self, d_alpha):
        self.alignment.alpha += d_alpha

    def rotate_beta(self, d_beta):
        self.alignment.alpha += d_beta

    def rotate_gamma(self, d_gamma):
        self.alignment.alpha += d_gamma

    def move_x(self, d_x):
        self.point.x += d_x

    def move_y(self, d_y):
        self.point.y += d_y

    def move_z(self, d_z):
        self.point.z += d_z
