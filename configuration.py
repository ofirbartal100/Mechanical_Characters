

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