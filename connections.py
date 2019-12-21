from abc import ABC
from abc import abstractmethod
from component import *

########
class Connection(ABC):
    '''
    represent a pin connection
    '''

    @abstractmethod
    def get_constraint(self):
        pass


class PinConnection(Connection):

    def __init__(self, comp1, comp2, joint1, joint2,rotation1,rotation2):
        self.comp1 = comp1
        self.comp2 = comp2
        self.joint1 = joint1
        self.joint2 = joint2
        self.rotation1 = rotation1
        self.rotation2 = rotation2

    def get_constraint(self):
        self.comp1.get_global(self.joint1) - self.comp1.
        self.comp1.get_global(self.joint1) - self.comp2.get_global(self.joint2)
