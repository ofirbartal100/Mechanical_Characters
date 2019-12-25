from abc import ABC
from abc import abstractmethod


class Component(ABC):
    """
    This class represents an abstract component object
    """

    id_counter = 0

    def __init__(self, configuration):
        self.configuration = configuration
        self.id = Component.generate_id()

    @staticmethod
    def generate_id():
        new_id = Component.id_counter
        Component.id_counter += 1
        return new_id

    @abstractmethod
    def get_global_position(self, local_joint_location):
        pass

    # @abstractmethod
    def get_local_position(self):
        pass
