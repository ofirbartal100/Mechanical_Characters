from abc import ABC


class Component(ABC):
    """
    This class represents an abstract component object
    """

    def __init__(self,configuration):
        self.configuration = configuration

    def get_global_position(self):
        pass

    def get_local_position(self):
        pass
