from abc import ABC, abstractmethod
import numpy as np

from component import *
from configuration import *


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


# TODO: change all contraints to retrurn a dict of resutls per variable

class PinConnection(Connection):

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
        Connection.__init__(self)

        self.comp1 = comp1
        self.comp2 = comp2
        self.joint1 = joint1
        self.joint2 = joint2
        self.rotation_axis1 = rotation_axis1
        self.rotation_axis2 = rotation_axis2
        self.derivs = None

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
        self.params[(self.comp1.id, 'alpha')] = 0
        self.params[(self.comp2.id, 'x')] = 0
        self.params[(self.comp2.id, 'y')] = 0
        self.params[(self.comp2.id, 'z')] = 0
        self.params[(self.comp2.id, 'alpha')] = 0

    def get_constraint(self):
        def const(position1x, position1y, position1z, alpha1,
                  position2x, position2y, position2z, alpha2):
            # update the positions of components
            self.params[(self.comp1.id, 'x')] = position1x
            self.params[(self.comp1.id, 'y')] = position1y
            self.params[(self.comp1.id, 'z')] = position1z
            self.params[(self.comp1.id, 'alpha')] = alpha1
            self.params[(self.comp2.id, 'x')] = position2x
            self.params[(self.comp2.id, 'y')] = position2y
            self.params[(self.comp2.id, 'z')] = position2z
            self.params[(self.comp2.id, 'alpha')] = alpha2
            # update the connection parts with the new values
            self.comp1.set_alpha(alpha1)
            self.comp2.set_alpha(alpha2)
            self.comp1.translate(Point(position1x, position1y, position1z))
            self.comp2.translate(Point(position2x, position2y, position2z))
            # calculate constraint values
            new_joint_global_location1 = self.comp1.local_vector_to_global(self.joint1)
            new_joint_global_location2 = self.comp2.local_vector_to_global(self.joint2)
            new_rotation_axis1_orientation = np.deg2rad(self.comp1.get_global_orientation(self.rotation_axis1))
            new_rotation_axis2_orientation = np.deg2rad(self.comp2.get_global_orientation(self.rotation_axis2))
            joint_dist_const = (new_joint_global_location1 - new_joint_global_location2) ** 2
            rotation_axis_const = (new_rotation_axis1_orientation - new_rotation_axis2_orientation) ** 2

            return [joint_dist_const[0],
                    joint_dist_const[1],
                    joint_dist_const[2],
                    rotation_axis_const[0],
                    rotation_axis_const[1],
                    rotation_axis_const[2],
                    ]

        return const

    def get_constraint_prime(self):
        def const_prime(x1, y1, z1, a1,
                        x2, y2, z2, a2):
            from numpy import cos, sin
            # rotation result on 3 axes
            # p + [x*(cos_b*cos_a) + y*(sin_g*sin_b*cos_a - cos_g*sin_a) + z*(sin_g*sin_a + cos_g*sin_b*cos_a),
            #      x*(cos_b*cos_a) + y*(sin_g*sin_b*sin_a + cos_g*cos_a) + z(-sin_g*cos_a + cos_g*sin_b*sin_a),
            #      x*(-sin_b) + y*(sin_g*cos_b) + z*(cos_g*cos_b)]
            # using only alpha
            # p + [x*cos_a - y*sin_a,
            #      x*sin_a + y*cos_a,
            #      z]
            # the actual constraint is (p1+v1 - (p2+v2))^2:
            # x: (x1 + (x_j*cos_a1 -y_j*sin_a1) - (x2 +(x_j*cos_a2 -y_j*sin_a2)))^2
            # y: (y1 + (x_j*sin_a1 + y_j*cos_a1) - (y2 +(x_j*sin_a + y_j*cos_a)))^2
            # z: (z1 + z_j - (z2 +z_j))^2
            # derivatives for line: new_joint_global_location1 = self.comp1.local_vector_to_global(self.joint1)

            new_joint_global_location1 = self.comp1.local_vector_to_global(self.joint1)
            new_joint_global_location2 = self.comp2.local_vector_to_global(self.joint2)
            der_x1 = 2 * (new_joint_global_location1[0] - new_joint_global_location2[0]) * 1
            der_y1 = 2 * (new_joint_global_location1[1] - new_joint_global_location2[1]) * 1
            der_z1 = 2 * (new_joint_global_location1[2] - new_joint_global_location2[2]) * 1
            der_a1 = (2 * (new_joint_global_location1 - new_joint_global_location2) *
                      np.array([-self.joint1[0] * sin(a1) - self.joint1[1] * cos(a1),
                                self.joint1[0] * cos(a1) - self.joint1[1] * sin(a1),
                                0])).sum()
            # derivatives for line: new_joint_global_location1 = self.comp1.local_vector_to_global(self.joint1)
            der_x2 = 2 * (new_joint_global_location1[0] - new_joint_global_location2[0]) * -1
            der_y2 = 2 * (new_joint_global_location1[1] - new_joint_global_location2[1]) * -1
            der_z2 = 2 * (new_joint_global_location1[2] - new_joint_global_location2[2]) * -1
            der_a2 = (2 * (new_joint_global_location1 - new_joint_global_location2) *
                      np.array([self.joint2[0] * sin(a2) + self.joint2[1] * cos(a2),
                                -self.joint2[0] * cos(a2) + self.joint2[1] * sin(a2),
                                0])).sum()
            # new_rotation_axis1_orientation = self.comp1.get_global_orientation(self.rotation_axis1)
            # new_rotation_axis2_orientation = self.comp2.get_global_orientation(self.rotation_axis2)
            # assuming 2d then the orientation is constant (always Z)
            # therefore it doesn't change the derivative
            return {(self.comp1.id, 'x'): der_x1,
                    (self.comp1.id, 'y'): der_y1,
                    (self.comp1.id, 'z'): der_z1,
                    (self.comp1.id, 'alpha'): der_a1,
                    (self.comp2.id, 'x'): der_x2,
                    (self.comp2.id, 'y'): der_y2,
                    (self.comp2.id, 'z'): der_z2,
                    (self.comp2.id, 'alpha'): der_a2}

        return const_prime


class PhaseConnection(Connection):

    def __init__(self, gear1, gear2, phase_diff=0, same_direc=False):
        Connection.__init__(self)
        self.gear1 = gear1
        self.gear2 = gear2
        self.phase_diff = phase_diff
        self.actuator = None
        self.direction = -1 if same_direc else 1
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
                self.gear1.rotate(Alignment(0, 0, alpha1))
                return [(alpha1 - self.actuator.get_alignment().alpha) ** 2]

            return const
        else:
            def const(alpha1, alpha2):
                self.params[(self.gear1.id, 'alpha')] = alpha1
                self.params[(self.gear2.id, 'alpha')] = alpha2
                self.gear1.set_alpha(alpha1)
                self.gear2.set_alpha(alpha2)
                return [(alpha1 - self.direction * self.gear1.get_phase_func(self.gear2)(
                    alpha2) + self.direction * self.phase_diff) ** 2]

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


class FixedConnection(Connection):

    def __init__(self, comp, fixed_position, fixed_orientation):
        """

        :param comp: component to fix
        :param position:
        :param orientation: Alignment object
                            alpha can be None, in that case the alpha remains free
        """
        Connection.__init__(self)

        self.comp = comp
        self.fixed_position = fixed_position
        self.fixed_orientation = fixed_orientation
        self.fix_alpha = fixed_orientation.alpha is not None

        # the order params are inserted must be same as order in const and cost_prime
        self.params[(self.comp.id, 'x')] = 0
        self.params[(self.comp.id, 'y')] = 0
        self.params[(self.comp.id, 'z')] = 0
        self.params[(self.comp.id, 'beta')] = 0
        self.params[(self.comp.id, 'gamma')] = 0
        if self.fix_alpha:
            self.params[(self.comp.id, 'alpha')] = 0

    def get_constraint(self):
        def const(positionx, positiony, positionz,
                  beta, gamma, alpha=None):
            # update the positions of components
            self.params[(self.comp.id, 'x')] = positionx
            self.params[(self.comp.id, 'y')] = positiony
            self.params[(self.comp.id, 'z')] = positionz
            self.params[(self.comp.id, 'beta')] = 0
            self.params[(self.comp.id, 'gamma')] = 0
            if self.fix_alpha:
                self.params[(self.comp.id, 'alpha')] = alpha
            # update the connection parts with the new values
            self.comp.translate(Point(positionx, positiony, positionz))
            self.comp.set_gamma(gamma)
            self.comp.set_beta(beta)
            if self.fix_alpha:
                self.comp.set_alpha(alpha)

            # calculate constraint values
            res = 0
            # position constraint
            res += ((np.array([positionx, positiony, positionz]) - self.fixed_position.vector()) ** 2).sum()
            xyz = ((np.array([positionx, positiony, positionz]) - self.fixed_position.vector()) ** 2)
            # angle constraints
            res += (gamma - self.fixed_orientation.gamma) ** 2
            res += (beta - self.fixed_orientation.beta) ** 2
            if self.fix_alpha:
                res += (alpha - self.fixed_orientation.alpha) ** 2
                return [xyz[0],
                        xyz[1],
                        xyz[2],
                        (gamma - self.fixed_orientation.gamma) ** 2,
                        (beta - self.fixed_orientation.beta) ** 2,
                        (alpha - self.fixed_orientation.alpha) ** 2]
            return [xyz[0],
                    xyz[1],
                    xyz[2],
                    (gamma - self.fixed_orientation.gamma) ** 2,
                    (beta - self.fixed_orientation.beta) ** 2]

        return const

    def get_constraint_prime(self):
        def const_prime(positionx, positiony, positionz,
                        beta, gamma, alpha=None):
            der_pos = 2 * (np.array([positionx, positiony, positionz]) - self.fixed_position.vector())
            der_dict = {(self.comp.id, 'x'): der_pos[0],
                        (self.comp.id, 'y'): der_pos[1],
                        (self.comp.id, 'z'): der_pos[2],
                        (self.comp.id, 'gamma'): 2 * (gamma - self.fixed_orientation.gamma),
                        (self.comp.id, 'beta'): 2 * (beta - self.fixed_orientation.beta)}
            if self.fix_alpha:
                der_dict[(self.comp.id, 'alpha')] = 2 * (alpha - self.fixed_orientation.alpha)
            return der_dict

        return const_prime
