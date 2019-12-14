from abc import ABC
from abc import abstractmethod
from typing import Union
import numpy as np
from component import *


class Connection(ABC):
    """
    represent a pin connection
    """

    @abstractmethod
    def get_constraint(self):
        pass

    @abstractmethod
    def get_free_param_count(self):
        pass

    @staticmethod
    def join_constraints(const_list: Union[list, 'Connection']):
        """
        generates a master constraint that can be optimized via Newton Raphson
        :param const_list:
        :return:
        """
        free_params_amount = Connection.free_params_in_assembly(const_list)
        def joint_const(param_list):
            """
            :param param_list: list of parameters for each constraint
            :return: sum of constraints parameterized with param_list
            """
            result = 0
            slice_start = 0
            for p, const in zip(free_params_amount, const_list):
                result += const.get_constraint()(*param_list[slice_start:slice_start+p])
                slice_start += p
            return result

        return joint_const

    @staticmethod
    def free_params_in_assembly(const_list):
        """

        :param const_list: list of constraints
        :return: an array of free the amount of free parameters in each constraint in the assembly
        """
        param_amounts = []
        for const in const_list:
            param_amounts.append(const.get_free_param_count())
        return np.array(param_amounts)


class PinConnection(Connection):

    def __init__(self, comp1, comp2, joint1, joint2):
        self.comp1 = comp1
        self.comp2 = comp2
        self.joint1 = joint1
        self.joint2 = joint2

    def get_constraint(self):
        self.comp1.get_global(self.joint1) - self.comp2.get_global(self.joint2)


class PhaseConnection(Connection):

    def __init__(self, gear1, gear2):
        self.gear1 = gear1
        self.gear2 = gear2
        self.actuator = None
        if self.gear1.radius == 0:
            self.actuator = self.gear1
        elif self.gear2.radius == 0:
            self.actuator = self.gear2

    def get_constraint(self):
        if self.actuator is not None:
            return lambda alpha1: (alpha1 - self.actuator.get_alignment().alpha) ** 2
        else:
            return lambda alpha1, alpha2: (alpha1 - self.gear1.get_phase_func(self.gear2)(alpha2))**2

    def get_free_param_count(self):
        if self.actuator is not None:
            return 1
        else:
            return 2

