from parts import *
from connections import *
import numpy as np
from collections import defaultdict
from scipy.optimize import minimize

import random
from configuration import *

class Assembly:
    id_counter = 0

    def __init__(self, connection_list):
        self.con_list = connection_list
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

    @abstractmethod
    def get_red_point(self):
        """
        :return: 3-dim position of the assembly red point in global axis
        """
        pass



def sample_radius_from_current(radius, diff_val=2,min_radius= 0.1):
    if radius<0.5:
        return max(min_radius, radius + round(random.uniform(-diff_val*radius, diff_val), 2))
    return max(min_radius,radius + round(random.uniform(-diff_val, diff_val), 2))


def sample_gear_parameters_from_current(gear_param,diff_val = 2):
    gear_param["radius"] = round(sample_radius_from_current(gear_param["radius"],diff_val = diff_val),2)
    return gear_param


def sample_length_from_current(length, diff_val=2, min_length = 0.1):
    if length<0.5:
        return max(min_length, length + round(random.uniform(-diff_val*length, diff_val), 2))
    return max(min_length,length + round(random.uniform(-diff_val, diff_val), 2))


def sample_stick_parameters_from_current(stick_param, diff_val=2):
    stick_param["length"] = round(sample_length_from_current(stick_param["length"], diff_val=diff_val),2)
    return stick_param


def sample_position(joint_location, diff_val=2):
    for i in range(3):
        joint_location[i] = round(random.uniform(-diff_val, diff_val), 2)
    return joint_location

def sample_point(point, diff_val=2):
    vector = point.vector()
    vector = sample_position(vector)
    return Point(*vector)

def sample_from_cur_assemblyA(assemblyA, gear_diff_val=2, stick_diff_val=2,position_diff_val = 2):
    config = assemblyA.config
    config["gear1_init_parameters"] = sample_gear_parameters_from_current(config["gear1_init_parameters"],gear_diff_val)
    config["gear2_init_parameters"] = sample_gear_parameters_from_current(config["gear2_init_parameters"],gear_diff_val)
    config["stick11_init_parameters"] = sample_stick_parameters_from_current(config["stick11_init_parameters"],stick_diff_val)
    config["stick12_init_parameters"] = sample_stick_parameters_from_current(config["stick12_init_parameters"],stick_diff_val)

    config["gear1_stick1_joint_location"] = sample_position(config["gear1_stick1_joint_location"],position_diff_val)
    #config["stick1_gear1_joint_location"] = sample_position(config["stick1_gear1_joint_location"],position_diff_val)
    config["gear2_stick2_joint_location"] = sample_position(config["gear2_stick2_joint_location"],position_diff_val)
    #config["stick2_gear2_joint_location"] = sample_position(config["stick2_gear2_joint_location"],position_diff_val)
    config["stick1_stick2_joint_location"] = sample_position(config["stick1_stick2_joint_location"],position_diff_val)
    #config["stick1_stick2_joint_location"] = sample_position(config["stick1_stick2_joint_location"],position_diff_val)

    config["gear1_fixed_position"] = sample_point(config["gear1_fixed_position"], position_diff_val)
    config["gear2_fixed_position"] = sample_point(config["gear2_fixed_position"], position_diff_val)

    return AssemblyA(config)


def is_vaild_assembleA(assemblyA):

    config = assemblyA.config
    #try:
        #assemblyA.update_state()

    if config["stick11_init_parameters"]["length"] < config["stick1_stick2_joint_location"][0]:

        print(f" stick length {config['stick11_init_parameters']['length']} and joint location is in {config['stick1_stick2_joint_location'][0]}")
        return False
    if config["stick12_init_parameters"]['length'] < config["stick2_stick1_joint_location"][0]:
        print(f" stick length {config['stick12_init_parameters']['length']} and joint location is in {config['stick2_stick1_joint_location'][0]}")

        return False

    joint_x, joint_y = config["gear1_fixed_position"].vector()[:2]
    center_x, center_y = config["gear1_stick1_joint_location"][:2]
    radius = config["gear1_init_parameters"]["radius"]


    if (joint_x - center_x)**2 +(joint_y - center_y)**2 > radius**2:
        print(
            f" center gear 1 {center_x,center_y} with radius {radius} and joint location is in {joint_x,joint_y}")
        return False

    joint_x, joint_y = config["gear2_fixed_position"].vector()[:2]
    center_x, center_y = config["gear2_stick2_joint_location"][:2]
    radius = config["gear2_init_parameters"]["radius"]
    if (joint_x - center_x) ** 2 + (joint_y - center_y) ** 2 > radius ** 2:
        print(
            f" center gear 2 {center_x,center_y} with radius {radius} and joint location is in {joint_x,joint_y}")
        return False

    #except Exception as e:
        #print(f"error = {e}")
        #return False

    return True

def get_assembly_curve(assembly):
    assembly_curve = []
    actuator = assembly.actuator
    for i in range(360):
        actuator.turn(i)
        assembly.update_state()
        assembly_curve.append(assembly.get_red_point_position())
    return assembly_curve


def ofir_score(curve1,curve2):
    return 0.5


def is_dissimilar(curve, database, gamma = 1):
    for dataset_curve in database:
        if ofir_score(curve,dataset_curve)<gamma:
            return False
    return True


def recursive_sample_assemblyA(assemblyA, curve_database = [], num_of_samples_around = 5):

    accepted_assemblies = []
    for i in range(num_of_samples_around):
        new_assemblyA = sample_from_cur_assemblyA(assemblyA)
        if is_vaild_assembleA(new_assemblyA):
            #assembly_curve = get_assembly_curve(new_assemblyA)
            assembly_curve = [1]
            if is_dissimilar(assembly_curve,curve_database):
                curve_database.append(assembly_curve)
                accepted_assemblies.append(new_assemblyA)

    return accepted_assemblies, curve_database


def return_prototype():
    config = dict()
    config["gear1_init_parameters"] = {"radius":2, "center":Point(0.0,0.0,0.0),"orientation": Alignment(0,0,0)}
    config["stick11_init_parameters"] = {"length": 6, "edge": Point(0,0,0), "orientation": Alignment(0,0,0)}
    config["gear2_init_parameters"] = {"radius": 1, "center": Point(10.0,10.0,0.0),"orientation": Alignment(0,0,0)}
    config["stick12_init_parameters"] = {"length": 4, "edge": Point(0.0,0.0,0.0), "orientation": Alignment(0,0,0)}

    config["gear1_fixed_position"] = Point(0.0,0.0,0.0)
    config["gear2_fixed_position"] = Point(10.0,10.0,0.0)

    config["gear1_fixed_orientation"] = Alignment(0,0,0)
    config["gear2_fixed_orientation"] = Alignment(0,0,0)

    config["gear1_stick1_joint_location"] = np.array([1,1,0],dtype=float)
    config["stick1_gear1_joint_location"] = np.array([0,0,0],dtype=float)
    config["gear2_stick2_joint_location"] = np.array([0.5,0.5,0],dtype=float)
    config["stick2_gear2_joint_location"] = np.array([0,0,0],dtype=float)

    config["stick1_stick2_joint_location"] = np.array([3,0,0],dtype=float)
    config["stick2_stick1_joint_location"] = np.array([4,0,0],dtype=float)
    return AssemblyA(config)

def create_assemblyA():
    new_assembly = sample_from_cur_assemblyA(return_prototype(), gear_diff_val=4, stick_diff_val=4,position_diff_val = 2)
    while not is_vaild_assembleA(new_assembly):
        print("not valid assembly")
        new_assembly = sample_from_cur_assemblyA(return_prototype(), gear_diff_val=4, stick_diff_val=4,position_diff_val = 2)
    return new_assembly

def create_assemblyA_database(min_samples_number=1000):
    origin_assembly = create_assemblyA()
    database = [origin_assembly]
    #curve_database = [get_assembly_curve(origin_assembly)]
    curve_database = [[1]]
    while len(database) < min_samples_number:
        accepted_assemblies, curve_database = recursive_sample_assemblyA(origin_assembly, curve_database)
        database+=accepted_assemblies
        while len(accepted_assemblies)>0 and len(database) < min_samples_number :
            origin_assembly = accepted_assemblies[0]
            neighbor_accepted_assemblies, curve_database = recursive_sample_assemblyA(origin_assembly, curve_database,
                                                                             max_number=min_samples_number - len(database))
            database += neighbor_accepted_assemblies
            accepted_assemblies = accepted_assemblies[1:]
            accepted_assemblies.append(neighbor_accepted_assemblies)


        origin_assembly = create_assemblyA()
        database.append(origin_assembly)
        #curve_database.append(get_assembly_curve(origin_assembly))
        curve_database.append([1])
    return database, curve_database


class AssemblyA:

    def __init__(self, config):
        self.config = config
        self._parse_config(self.config)

    def _parse_config(self, config):
        self.actuator = Actuator()
        self.components = [Gear(**config["gear1_init_parameters"]), Stick(**config["stick11_init_parameters"]),
                           Gear(**config["gear2_init_parameters"]), Stick(**config["stick12_init_parameters"])]

        self.connections = [PhaseConnection(self.actuator, self.components[0]),
                            PhaseConnection(self.components[0], self.components[2]),

                            FixedConnection(self.components[0], config["gear1_fixed_position"], config["gear1_fixed_orientation"]),
                            FixedConnection(self.components[0], config["gear2_fixed_position"], config["gear2_fixed_orientation"]),

                            PinConnection(self.components[0], self.components[1], config["gear1_stick1_joint_location"],
                                          config["stick1_gear1_joint_location"]),
                            PinConnection(self.components[2], self.components[3], config["gear2_stick2_joint_location"],
                                          config["stick2_gear2_joint_location"]),
                            PinConnection(self.components[1], self.components[3],
                                          config["stick1_stick2_joint_location"],
                                          config["stick1_stick2_joint_location"])]

        self.red_point_component = self.components[1]

    def get_constraints(self):
        C = lambda s: 0
        for connection in self.connections:
            C += connection.get_constraint()
        return C

    def get_red_point_position(self):
        """
        :return: 3-dim position of the assembly red point in global axis
        """
        return self.red_point_component.get_global_position(np.array([self.red_point_component.length, 0, 0]))



database, curve_databas = create_assemblyA_database(10)
print(len(database))
print(len(curve_databas))

# config = create_assemblyA().config

config = database[5].config
print("------------")
for key1 in config:
    print(key1)
    if type(config[key1])== type(dict()):
        for key in config[key1]:
            print(key)
            print(config[key1][key])
    else:
        print(config[key1])