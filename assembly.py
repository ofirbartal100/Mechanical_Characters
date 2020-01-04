from parts import *
from connections import *
import numpy as np


class Assembly:
    id_counter = 0

    def __init__(self, connection_list, newt_iters=100, newt_tolerance=1e-6):
        self.con_list = connection_list
        self.iterations = newt_iters
        self.tolerance = newt_tolerance
        self.const, self.param_index = self.get_assembly_constraint()
        self.const_deriv = self.get_assembly_constraints_deriv()
        self.cur_state = self.free_params_in_assembly()
        # make sure the assembly is valid
        self.update_state()
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

    def update_state(self):
        """
        update state of assembly to
        :return:
        """
        converged = False
        x = self.get_cur_state_array()
        for n in range(self.iterations):
            f = self.const(x)
            df = self.const_deriv(x)

            if abs(f) < self.tolerance:  # exit function if we're close enough
                converged = True
                break

            x = x - df * f / np.linalg.norm(df) ** 2  # update guess
        if converged:
            self.update_cur_state_from_array(x)
        else:
            raise (RuntimeError('failed to update state, illegal assembly or not enough iterations'))

    def update_cur_state_from_array(self, new_state_array):
        for param, idx in self.param_index.items():
            self.cur_state[param] = new_state_array[idx]

    def get_cur_state_array(self):
        return np.array([self.cur_state[k] for k in self.param_index])


class AssemblyA:

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
