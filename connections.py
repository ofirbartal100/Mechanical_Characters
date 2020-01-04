from collections import defaultdict
from typing import Union

from component import *


class Connection(ABC):
    """
    represent a pin connection
    """

    id_counter = 0

    def __init__(self):
        self.params = {}
        self.id = Connection.id_counter
        Connection.id_counter += 1

    def get_free_params(self):
        return self.params

    def get_free_params_cnt(self):
        return len(self.params)

    def get_id(self):
        return self.id

    @abstractmethod
    def get_constraint(self):
        pass


class PinConnection(Connection):

    def __init__(self, comp1, comp2, joint1, joint2, rotation1, rotation2):
        self.comp1 = comp1
        self.comp2 = comp2
        self.joint1 = joint1
        self.joint2 = joint2
        self.rotation1 = rotation1
        self.rotation2 = rotation2

    def get_constraint(self):
        # self.comp1.get_global(self.joint1) - self.comp1.
        self.comp1.get_global(self.joint1) - self.comp2.get_global(self.joint2)


class PhaseConnection(Connection):

    def __init__(self, gear1, gear2):
        Connection.__init__(self)
        self.gear1 = gear1
        self.gear2 = gear2
        self.actuator = None
        if self.gear1.radius == 0:
            self.actuator = self.gear1
            self.gear1 = self.gear2
        elif self.gear2.radius == 0:
            self.actuator = self.gear2
        # add the free parameters
        if self.actuator is not None:
            self.params[(self.gear1.id, 'alpha')] = 0
        else:
            self.params[(self.gear1.id, 'alpha')] = 0
            self.params[(self.gear2.id, 'alpha')] = 0

    def get_constraint(self):
        if self.actuator is not None:
            def const(alpha1):
                self.params[(self.gear1.id, 'alpha')] = alpha1
                return (alpha1 - self.actuator.get_alignment().alpha) ** 2

            return const
        else:
            def const(alpha1, alpha2):
                self.params[(self.gear1.id, 'alpha')] = alpha1
                self.params[(self.gear2.id, 'alpha')] = alpha2
                return (alpha1 - self.gear1.get_phase_func(self.gear2)(alpha2)) ** 2

            return const

    def get_constraint_prime(self):
        if self.actuator is not None:
            # (alpha1 - self.actuator.get_alignment().alpha) ** 2
            return lambda alpha1: {(self.gear1.id, 'alpha'): (alpha1 - self.actuator.get_alignment().alpha) * 2}
        else:
            # (alpha1 - self.gear1.get_phase_func(self.gear2)(alpha2)) ** 2
            return lambda alpha1, alpha2: {
                (self.gear1.id, 'alpha'): (alpha1 - self.gear1.get_phase_func(self.gear2)(alpha2)) * 2,
                (self.gear2.id, 'alpha'): ((alpha1 - self.gear1.get_phase_func(self.gear2)(
                    alpha2)) * 2) * - self.gear1.get_phase_func(self.gear2)(1)}
