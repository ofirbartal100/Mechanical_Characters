from abc import ABC
from abc import abstractmethod


class Component(ABC):
    '''
    This class represents an abstract component object
    '''

    def __init__(self, configuration):
        self.configuration = configuration

    @abstractmethod
    def get_global_position(self, local_joint_location):
        pass

    # @abstractmethod
    def get_local_position(self):
        pass
