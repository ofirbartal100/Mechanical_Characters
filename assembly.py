from parts import *
from connections import *
import numpy as np
from collections import defaultdict
from scipy.optimize import minimize
from matplotlib import pyplot as plt


class Assembly:
    id_counter = 0

    def __init__(self, connection_list):
        self.con_list = connection_list
        self.const, self.param_index = self.get_assembly_constraint()
        self.const_deriv = self.get_assembly_constraints_deriv()
        self.cur_state = self.free_params_in_assembly()
        # make sure the assembly is valid
        if not self.update_state():
            raise Exception("assembly failed to init")
        self.id = Assembly.id_counter
        Assembly.id_counter += 1

    def get_assembly_constraint(self):
        """
        generates a master constraint that can be optimized via Newton Raphson
        :param connection_list:
        :return: the constrain describing the whole assemply
                and the index (dict(param:position)) of the params
        """
        param_index = {p: i for i, p in enumerate(self.free_params_in_assembly())}

        def assembly_const(param_list):
            """
            :param param_list: list of parameters for each constraint
                               must be ordered according to param_index
            :return: sum of constraints parameterized with param_list
            """
            result = 0
            for i, con in enumerate(self.con_list):
                result += con.get_constraint()(*[param_list[param_index[p]] for p in con.get_free_params()])
            return result

        return assembly_const, param_index

    def get_assembly_constraints_deriv(self):
        """
        generates a master constraint that can be optimized via Newton Raphson
        :param connection_list:
        :return: the constrain describing the whole assemply
                and the index (dict(param:position)) of the params
        """

        def assembly_const_deriv(param_list):
            """
            :param param_list: list of parameters for each constraint
                               must be ordered according to param_index
            :return: sum of constraints parameterized with param_list
            """
            joint_grad_dict = defaultdict(lambda: 0)
            for i, con in enumerate(self.con_list):
                gradient_dict = con.get_constraint_prime()(
                    *[param_list[self.param_index[p]] for p in con.get_free_params()])
                # why is the gradient on y?
                for k in gradient_dict:
                    joint_grad_dict[k] += gradient_dict[k]
            # order the result according to param_index
            result = np.zeros(len(joint_grad_dict))
            for k in joint_grad_dict:
                result[self.param_index[k]] = joint_grad_dict[k]
            return result

        return assembly_const_deriv

    def free_params_in_assembly(self):
        """
        :param connection_list: list of constraints
        :return: a set containing tuples specifying the free params in the assembly
        """
        params = {}
        for const in self.con_list:
            params.update(const.get_free_params())
        return params

    def free_params_cnt_in_assembly(self):
        """

        :param connection_list: list of constraints
        :return: a set containing tuples specifying the free params in the assembly
        """
        params = {}
        for const in self.con_list:
            params.update(const.get_free_params())
        return len(params)

    # def update_state(self):
    #     """
    #     update state of assembly to
    #     :return:
    #     """
    #     converged = False
    #     x = self.get_cur_state_array()
    #     for n in range(self.iterations):
    #         f = self.const(x)
    #         state_f = self.get_state_from_array(x)
    #         df = self.const_deriv(x)
    #         state_df = self.get_state_from_array(df)
    #         # deriv on alpha should cancel in direction
    #         if abs(f) < self.tolerance:  # exit function if we're close enough
    #             converged = True
    #             break
    #
    #         x = x - df * f / np.linalg.norm(df) ** 2  # update guess
    #     if converged:
    #         self.update_cur_state_from_array(x)
    #     else:
    #         raise (RuntimeError('failed to update state, illegal assembly or not enough iterations'))

    def update_state(self):
        '''

        :return: True/False to indicate convergance
        '''
        x = self.get_cur_state_array()
        res = minimize(self.const, x)
        if res.success:
            self.update_cur_state_from_array(res['x'])
            return True
        else:
            return False

    def update_cur_state_from_array(self, new_state_array):
        for param, idx in self.param_index.items():
            self.cur_state[param] = new_state_array[idx]

    def get_state_from_array(self, state_array):
        state = {}
        for param, idx in self.param_index.items():
            state[param] = state_array[idx]
        return state

    def get_cur_state_array(self):
        return np.array([self.cur_state[k] for k in self.param_index])


class AssemblyA:

    def plot_assembly(self):
        fig, ax = plt.subplots()
        for comp in self.components:
            if isinstance(comp, Stick):
                edge1 = comp.configuration.position.vector()[:2]
                edge2 = comp.get_global_position(Point(comp.length, 0, 0))[:2]
                ax.plot(edge1, edge2, '-')
            if isinstance(comp, Gear):
                center = comp.configuration.position.vector()[:2]
                radius = comp.radius
                direction = comp.get_global_position(Point(comp.radius, 0, 0))[:2]
                ax.add_artist(plt.Circle(center, radius))
                ax.plot(center, direction, '-')
        fig.show()

    def __init__(self, config):
        self.config = config
        self._parse_config(self.config)

    def _parse_config(self, config):
        self.components = [Gear(), Stick(), Gear(), Stick()]
        self.connections = [PinConnection(self.components[0], self.components[1], 0.5, (0, 0)),
                            PinConnection(self.components[2], self.components[3], 0.5, (0, 0)),
                            PinConnection(self.components[1], self.components[3], (1, 1), (0.5, 0.5))]

    def get_constraints(self):
        C = lambda s: 0
        for connection in self.connections:
            C += connection.get_constraint()
        return C

    # create binned data
