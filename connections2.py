from abc import ABC, abstractmethod
import numpy as np

from component import *
from configuration import *
from scipy.spatial.transform import Rotation as R


class Connection2(ABC):
    """
    represent a pin connection
    """

    id_counter = 0

    def __init__(self):
        self.params = {}
        self.id = Connection2.id_counter
        Connection2.id_counter += 1

    def get_free_params(self):
        return self.params

    def get_free_params_cnt(self):
        return len(self.params)

    def get_id(self):
        return self.id

    @abstractmethod
    def get_constraint_by_the_book(self):
        pass


# TODO: change all contraints to retrurn a dict of resutls per variable

class PinConnection2(Connection2):

    def __init__(self, comp1, comp2, joint1, joint2, rotation_axis1=Alignment(90, 90, 0),
                 rotation_axis2=Alignment(90, 90, 0)):
        """

        :param comp1: component 1 involved in the pin connection
        :param comp2: component 1 involved in the pin connection
        :param joint1: a vector: np array of shape (3,)
                       specifies the joint location in local coordinates for comp1
                       this is also the orientation as the origin is an edge
        :param joint2: a vector: np array of shape (3,)
                       specifies the joint location in local coordinates for comp2
                       this is also the orientation as the origin is an edge
        :param rotation_axis1: an Alignment object
                               rotation axis of the pin (imagined as the vector describing the pin)
                               in local coordinates
        :param rotation_axis2: an Alignment object
                               rotation axis of the pin (imagined as the vector describing the pin)
                               in local coordinates
        """
        Connection2.__init__(self)

        self.comp1 = comp1
        self.comp2 = comp2
        self.joint1 = joint1
        self.joint2 = joint2
        self.rotation_axis1 = rotation_axis1
        self.rotation_axis2 = rotation_axis2

        if isinstance(joint1, Point):
            self.joint1 = joint1.vector()
        if isinstance(joint2, Point):
            self.joint2 = joint2.vector()
        if isinstance(rotation_axis1, Alignment):
            self.rotation_axis1 = rotation_axis1.vector()
        if isinstance(rotation_axis2, Alignment):
            self.rotation_axis2 = rotation_axis2.vector()

        # the order params are inserted must be same as order in const and cost_prime
        self.params[(self.comp1.id, 'x')] = 0
        self.params[(self.comp1.id, 'y')] = 0
        self.params[(self.comp1.id, 'z')] = 0
        self.params[(self.comp1.id, 'gamma')] = 0
        self.params[(self.comp1.id, 'beta')] = 0
        self.params[(self.comp1.id, 'alpha')] = 0

        self.params[(self.comp2.id, 'x')] = 0
        self.params[(self.comp2.id, 'y')] = 0
        self.params[(self.comp2.id, 'z')] = 0
        self.params[(self.comp2.id, 'gamma')] = 0
        self.params[(self.comp2.id, 'beta')] = 0
        self.params[(self.comp2.id, 'alpha')] = 0

    def get_constraint_by_the_book(self):
        # should get 12 state variables
        def const(x0, y0, z0, c0, b0, a0, x1, y1, z1, c1, b1, a1):
            r0 = R.from_euler('xyz', [c0, b0, a0]).as_matrix()
            r1 = R.from_euler('xyz', [c1, b1, a1]).as_matrix()

            X = (np.array([x0, y0, z0]) + r0 @ self.joint1) - (np.array([x1, y1, z1]) + r1 @ self.joint2)

            V = (r0 @ self.rotation_axis1) - (r1 @ self.rotation_axis2)
            return [*X, *V]

        return const, self.params

    def get_constraint_prime_by_the_book(self):
        def const_prime(x0, y0, z0, c0, b0, a0, x1, y1, z1, c1, b1, a1):
            # gradients for X by first component
            r0xy = R.from_euler('xy', [c0, b0]).as_matrix()
            r0x = R.from_euler('x', c0).as_matrix()
            r0z = R.from_euler('z', a0).as_matrix()
            r0yz = R.from_euler('yz', [b0, a0]).as_matrix()

            r0doa0 = np.array([[-np.sin(a0), np.cos(a0), 0],
                               [-np.cos(a0), -np.sin(a0), 0],
                               [0, 0, 0]])

            r0dob0 = np.array([[-np.sin(b0), 0, np.cos(b0)],
                               [0, 0, 0],
                               [-np.cos(b0), 0, -np.sin(b0)]])

            r0doc0 = np.array([[0, 0, 0],
                               [0, -np.sin(c0), np.cos(c0)],
                               [0, -np.cos(c0), -np.sin(c0)]])

            doa0 = (r0xy @ r0doa0) @ self.joint1
            dob0 = (r0x @ r0dob0 @ r0z) @ self.joint1
            doc0 = (r0doc0 @ r0yz) @ self.joint1

            # gradients for X by second component
            r1xy = R.from_euler('xy', [c1, b1]).as_matrix()
            r1x = R.from_euler('x', c1).as_matrix()
            r1z = R.from_euler('z', a1).as_matrix()
            r1yz = R.from_euler('yz', [b1, a1]).as_matrix()

            r1doa1 = np.array([[-np.sin(a1), np.cos(a1), 0],
                               [-np.cos(a1), -np.sin(a1), 0],
                               [0, 0, 0]])

            r1dob1 = np.array([[-np.sin(b1), 0, np.cos(b1)],
                               [0, 0, 0],
                               [-np.cos(b1), 0, -np.sin(b1)]])

            r1doc1 = np.array([[0, 0, 0],
                               [0, -np.sin(c1), np.cos(c1)],
                               [0, -np.cos(c1), -np.sin(c1)]])

            doa1 = (r1xy @ r1doa1) @ self.joint2
            dob1 = (r1x @ r1dob1 @ r1z) @ self.joint2
            doc1 = (r1doc1 @ r1yz) @ self.joint2


            # V derivatives
            Vdoa0 = (r0xy @ r0doa0) @ self.rotation_axis1
            Vdob0 = (r0x @ r0dob0 @ r0z) @ self.rotation_axis1
            Vdoc0 = (r0doc0 @ r0yz) @ self.rotation_axis1

            Vdoa1 = (r1xy @ r1doa1) @ self.rotation_axis2
            Vdob1 = (r1x @ r1dob1 @ r1z) @ self.rotation_axis2
            Vdoc1 = (r1doc1 @ r1yz) @ self.rotation_axis2


            deriv = np.zeros((6, 12))
            deriv[0, :] = np.array([1, 0, 0, dob0[0], doc0[0], doa0[0],
                                    -1, 0, 0, -doa1[0], -dob1[0], -doc1[0]])
            deriv[1, :] = np.array([0, 1, 0, dob0[1], doc0[1], doa0[1],
                                    0, -1, 0, -doa1[1], -dob1[1], -doc1[1]])
            deriv[2, :] = np.array([0, 0, 1, dob0[2], doc0[2], doa0[2],
                                    0, 0, -1, -doa1[2], -dob1[2], -doc1[2]])
            deriv[3, :] = np.array([0, 0, 0, Vdoa0[0], Vdob0[0], Vdoc0[0],
                                    0, 0, 0, -Vdoa1[0], -Vdob1[0], -Vdoc1[0]])
            deriv[4, :] = np.array([0, 0, 0, Vdoa0[1], Vdob0[1], Vdoc0[0],
                                    0, 0, 0, -Vdoa1[1], -Vdob1[1], -Vdoc1[1]])
            deriv[5, :] = np.array([0, 0, 0, Vdoa0[2], Vdob0[2], Vdoc0[0],
                                    0, 0, 0, -Vdoa1[2], -Vdob1[2], -Vdoc1[2]])
            return deriv

        return const_prime, self.params


# dont use! we dont bind 2 gears together
class PhaseConnection2(Connection2):

    def __init__(self, gear1, gear2, phase_diff=0):
        Connection2.__init__(self)
        self.gear1 = gear1
        self.gear2 = gear2
        self.phase_diff = phase_diff
        self.actuator = None
        if self.gear1.radius == 0:
            self.actuator = self.gear1
            self.gear1 = self.gear2
        elif self.gear2.radius == 0:
            self.actuator = self.gear2
        # add the free parameters

        self.params[(self.gear1.id, 'x')] = 0
        self.params[(self.gear1.id, 'y')] = 0
        self.params[(self.gear1.id, 'z')] = 0
        self.params[(self.gear1.id, 'gamma')] = 0
        self.params[(self.gear1.id, 'beta')] = 0
        self.params[(self.gear1.id, 'alpha')] = 0

        if self.actuator is None:
            self.params[(self.gear2.id, 'x')] = 0
            self.params[(self.gear2.id, 'y')] = 0
            self.params[(self.gear2.id, 'z')] = 0
            self.params[(self.gear2.id, 'gamma')] = 0
            self.params[(self.gear2.id, 'beta')] = 0
            self.params[(self.gear2.id, 'alpha')] = 0

    def get_constraint_by_the_book(self):
        # should get 12 state variables
        def const(x0, y0, z0, a0, b0, c0, x1, y1, z1, a1, b1, c1):
            # self.gear1.apply_state(x0, y0, z0, a0, b0, c0)
            # self.gear2.apply_state(x1, y1, z1, a1, b1, c1)
            r = self.gear2.num_of_teeth / self.gear1.num_of_teeth

            return [a0 - r * (a1 + self.phase_diff)]

        # should get 6 state variables
        def const_with_actuator(x0, y0, z0, a0, b0, c0):
            return [a0 - self.actuator.get_phase()]

        if self.actuator:
            return const_with_actuator, self.params
        else:
            return const, self.params

    # needs to multiply every partial derivative by get_constraint_by_the_book when calculating overall derive
    def get_constraint_prime_by_the_book(self):
        def const_prime(x0, y0, z0, a0, b0, c0, x1, y1, z1, a1, b1, c1):
            r = self.gear2.num_of_teeth / self.gear1.num_of_teeth
            return [0, 0, 0, 1, 0, 0, 0, 0, 0, -r, 0, 0]

        def const_prime_with_actuator(x0, y0, z0, a0, b0, c0):
            return [0, 0, 0, 1, 0, 0]

        if self.with_actuator:
            return const_prime_with_actuator, self.params
        else:
            return const_prime, self.params


class FixedConnection2(Connection2):

    def __init__(self, comp, fixed_position, fixed_orientation):
        """

        :param comp: component to fix
        :param position:
        :param orientation: Alignment object
                            alpha can be None, in that case the alpha remains free
        """
        Connection2.__init__(self)

        self.comp = comp
        self.fixed_position = fixed_position
        self.fixed_orientation = fixed_orientation

        # the order params are inserted must be same as order in const and cost_prime
        self.params[(self.comp.id, 'x')] = 0
        self.params[(self.comp.id, 'y')] = 0
        self.params[(self.comp.id, 'z')] = 0
        self.params[(self.comp.id, 'gamma')] = 0
        self.params[(self.comp.id, 'beta')] = 0
        self.params[(self.comp.id, 'alpha')] = 0

    def get_constraint_by_the_book(self):
        # should get 12 state variables
        def const(x0, y0, z0, a0, b0, c0):
            # self.comp.apply_state(x0, y0, z0, a0, b0, c0)
            X = np.array([x0, y0, z0]) - self.fixed_position
            V = np.array([a0, b0, c0]) - self.fixed_orientation
            return [*X, *V]

        return const, self.params

    # needs to multiply every partial derivative by get_constraint_by_the_book when calculating overall derive
    def get_constraint_prime_by_the_book(self):
        def const_prime(x0, y0, z0, a0, b0, c0):
            return np.eye(6)

        return const_prime, self.params
