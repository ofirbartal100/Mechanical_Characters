from abc import ABC
from configuration import *


class Component(ABC):
    """
    This class represents an abstract component object
    """

    id_counter = 0

    def __init__(self, position, alignment):
        self.configuration = Configuration(position, alignment)
        self.id = Component.generate_id()

    @staticmethod
    def generate_id():
        new_id = Component.id_counter
        Component.id_counter += 1
        return new_id

    def get_global_orientation(self, local_alignment):
        """
        get global orientation in Euler angles
        :param local_alignment: an Alignment object or vector of eurler angles
        :return: rotated Alignment object
        """
        if isinstance(local_alignment, Alignment):
            local_alignment = local_alignment.vector()

        return self.configuration.alignment.vector() + local_alignment

    def get_global_position(self, local_position):
        """
        get global orientation in cartesian coordinates
        :param local_position: a coordinate vector
        :return: a translated Posiotion object
        """
        return self.configuration.position.vector() + local_position

    def local_vector_to_global(self, vec):
        """
        rotates and translates a local vector to it's global position
        :param vec: np array of shape (3,)
        :return: np array of shape (3,)
        """
        rotation = self.configuration.alignment.get_rotation_obj()
        res = rotation.apply(vec)
        res = self.configuration.position.vector() + res
        return res

    # @abstractmethod
    def get_local_position(self):
        pass

    def rotate(self, new_alignment):
        self.configuration.alignment = new_alignment

    def translate(self, new_position):
        self.configuration.position = new_position

    def set_alpha(self, new_alpha):
        self.configuration.alignment.alpha = new_alpha

    def set_gamma(self, new_gamma):
        self.configuration.alignment.gamma = new_gamma

    def set_beta(self, new_beta):
        self.configuration.alignment.beta = new_beta
