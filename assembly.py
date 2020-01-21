from parts import *
from connections2 import *
from curve import *
from collections import defaultdict
from scipy.optimize import minimize
from matplotlib import pyplot as plt
import dill as pickle
from sklearn.decomposition import PCA
# import pickle

import random
from configuration import *

image_num = 0


def describe_comp(comp):
    comp_desc = {'type': str(type(comp)).split(".")[1].split("'")[0],
                 'id': comp.id,
                 'length': comp.radius if isinstance(comp, Gear) else comp.length,
                 'position': comp.configuration.position.vector().tolist(),
                 'orientation': comp.configuration.alignment.vector().tolist()}
    return comp_desc


class Assembly:
    id_counter = 0

    def __init__(self, connection_list, components, actuator=None, iters=100, tol=1e-4, plot_newt=False):
        self.components = components
        self.con_list = connection_list
        self.iterations = iters
        self.tolerance = tol
        self.actuator = actuator
        # self.const, self.param_index = self.get_assembly_constraint()
        self.const, self.param_index = self.get_assembly_constraint2()
        # self.const_deriv = self.get_assembly_constraints_deriv()
        self.const_deriv = self.get_assembly_constraints_deriv2()
        self.cur_state = self.free_params_in_assembly()
        self.plot_newt = plot_newt

        # make sure the assembly is valid
        # if not self.update_state():
        self.update_state2()
        #     print("Failed")
        # raise Exception("assembly failed to init")
        self.id = Assembly.id_counter
        Assembly.id_counter += 1

    def merge_assembly(self, other_asm):
        """
        returns a new assembly made of self and other assembly
        :return:
        """
        return Assembly(self.con_list + other_asm.con_list,
                        components=self.components + other_asm.components,
                        actuator=other_asm.actuator or self.actuator,
                        iters=self.iterations,
                        tol=self.tolerance,
                        plot_newt=self.plot_newt)

    def describe_assembly(self):
        return [describe_comp(c) for c in self.components]

    def plot_assembly(self, plot_path=None, image_number=None, save_images=False, user_fig=None):
        fig, ax = plt.subplots()
        for comp in self.components:
            if isinstance(comp, Stick):
                clr = 'r'
                if user_fig is not None:
                    if comp in user_fig.components:
                        clr = 'k'
                edge1 = comp.configuration.position.vector()[:2]
                edge2 = comp.get_global_position(Point(comp.length, 0, 0))[:2]
                ax.plot((edge1[0], edge2[0]), (edge1[1], edge2[1]), f'-{clr}', alpha=0.5, linewidth=2)
            if isinstance(comp, Gear):
                clr = 'y'
                if user_fig is not None:
                    if comp in user_fig.components:
                        clr = 'k'
                center = comp.configuration.position.vector()[:2]
                radius = comp.radius
                direction = comp.get_global_position(Point(comp.radius, 0, 0))[:2]
                plot_circle(ax, center[0], center[1], radius)
                ax.plot((center[0], direction[0]), (center[1], direction[1]), f'{clr}-', alpha=0.5, linewidth=2)
        plt.xlim(-20, 50)
        plt.ylim(-50, 50)
        plt.grid(linestyle='--')
        ax.set_aspect('equal')
        # fig.show()
        if plot_path and save_images:
            plt.savefig(plot_path + fr"{image_number}")

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
            result = []
            for i, con in enumerate(self.con_list):
                connection_const_res = con.get_constraint()(
                    *[param_list[param_index[p]] for p in con.get_free_params()])
                # result += connection_const_res
                result = result + connection_const_res
                # self.plot_assembly()
            return sum(result)

        return assembly_const, param_index

    def get_assembly_constraint2(self):
        """
        generates a master constraint that can be optimized via Newton Raphson
        :param connection_list:
        :return: the constrain describing the whole assemply
                and the index (dict(param:position)) of the params
        """
        self.param_index = {p: i for i, p in enumerate(self.free_params_in_assembly())}

        def assembly_const(param_list):
            """
            :param param_list: list of parameters for each constraint
                               must be ordered according to param_index
            :return: sum of constraints parameterized with param_list
            """
            result = []
            for i, con in enumerate(self.con_list):
                connection_const_res = con.get_constraint_by_the_book()[0](
                    *[param_list[self.param_index[p]] for p in con.get_free_params()])
                # result += connection_const_res
                result = result + connection_const_res
                # self.plot_assembly()
            self.C = np.array(result)
            return 0.5 * np.array(result) @ np.array(result)

        return assembly_const, self.param_index

    def get_assembly_constraints_deriv2(self):
        """
        generates a master constraint that can be optimized via Newton Raphson
        :param connection_list:
        :return: the constrain describing the whole assemply
                and the index (dict(param:position)) of the params
        """
        self.const(np.zeros(len(self.components) * 6))

        def assembly_const_deriv(param_list):
            """
            :param param_list: list of parameters for each constraint
                               must be ordered according to param_index
            :return: sum of constraints parameterized with param_list
            """
            doCdoSt = np.zeros((len(self.C), len(self.param_index)))
            row = 0
            for i, con in enumerate(self.con_list):
                gradient = con.get_constraint_prime_by_the_book()[0](
                    *[param_list[self.param_index[p]] for p in con.get_free_params()])
                gradient = np.array(gradient)
                # 1D
                if gradient.size == gradient.shape[0]:
                    for j, p in enumerate(con.get_free_params()):
                        doCdoSt[row, self.param_index[p]] = gradient[j]
                    row += 1

                else:
                    for l in range(gradient.shape[0]):
                        for j, p in enumerate(con.get_free_params()):
                            doCdoSt[row + l, self.param_index[p]] = gradient[l, j]
                    row += gradient.shape[0]

                # C = con.get_constraint_by_the_book()[0](*[param_list[self.param_index[p]] for p in con.get_free_params()])
                # if len(C) > 1:
                #     local_grad = np.array(C) @ gradient
                # else:
                #     local_grad = np.array(C) * gradient
                # for j,p in enumerate(con.get_free_params()):
                #     res[self.param_index[p]] += local_grad[j]

            return self.C @ doCdoSt

        return assembly_const_deriv

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
    #     # 1a,2a,1x,1y,1z,1b,1g,2x,2y,2z,2b,2g,3x,3y,3z,3a
    #     for n in range(self.iterations):
    #         f = self.const(x)
    #         if self.plot_newt:
    #             self.plot_assembly()
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
    #         return True
    #     else:
    #         return False

    def update_state(self):
        '''

        :return: True/False to indicate convergance
        '''
        x = self.get_cur_state_array()
        res = minimize(self.const, x, method='Powell')
        if res.success:
            self.update_cur_state_from_array(res['x'])
            x = self.get_cur_state_array()
            # print(self.const(x))
            return True
        else:
            return False

    def update_state2(self):
        '''

        :return: True/False to indicate convergance
        '''
        x = self.get_cur_state_array()
        res = minimize(self.const, x, method='BFGS', jac=self.const_deriv)
        if res.success:
            self.update_cur_state_from_array(res['x'])
            return True
        else:
            # change starting guess
            if x.mean() == 0:
                self.update_cur_state_from_array(res['x'])
            return False

    def update_cur_state_from_array(self, new_state_array):
        for param, idx in self.param_index.items():
            self.cur_state[param] = new_state_array[idx]
        for i, comp in enumerate(self.components):
            self.components[i] = self.update_comp(comp)

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

    def update_comp(self, comp):
        i = comp.id
        if (i, 'x') in self.cur_state:
            comp.configuration.position.x = self.cur_state[(i, 'x')]
        if (i, 'y') in self.cur_state:
            comp.configuration.position.y = self.cur_state[(i, 'y')]
        if (i, 'z') in self.cur_state:
            comp.configuration.position.z = self.cur_state[(i, 'z')]
        if (i, 'alpha') in self.cur_state:
            comp.configuration.alignment.alpha = self.cur_state[(i, 'alpha')]
        if (i, 'beta') in self.cur_state:
            comp.configuration.alignment.beta = self.cur_state[(i, 'beta')]
        if (i, 'gamma') in self.cur_state:
            comp.configuration.alignment.gamma = self.cur_state[(i, 'gamma')]
        return comp


def sample_radius_from_current(radius, diff_val=2, min_radius=0.1):
    if radius < 0.5:
        return round(max(min_radius, radius + random.uniform(-diff_val * radius, diff_val)), 2)
    return round(max(min_radius, radius + random.uniform(-diff_val, diff_val)), 2)


def sample_gear_parameters_from_current(gear_param, diff_val=2,second_gear = False, gear1_radius = 0.0):
    if second_gear:
        assert gear1_radius>0
        power = random.choice([-1,0,1])
        num =random.choice([2,3])
        gear_param["radius"] = gear1_radius*(num**power)
    else:
        gear_param["radius"] = round(sample_radius_from_current(gear_param["radius"], diff_val=diff_val), 2)
    return gear_param


def sample_length_from_current(length, diff_val=2, min_length=0.1):
    if length < 0.5:
        return round(max(min_length, length + random.uniform(-diff_val * length, diff_val)), 2)
    return round(max(min_length, length + random.uniform(-diff_val, diff_val)), 2)


def sample_stick_parameters_from_current(stick_param, diff_val=2, stick2_len_params=None):
    if stick2_len_params:
        radius1, radius2, gears_dis, stick1_part_len = stick2_len_params
        min_val = gears_dis + radius1 + radius2 - stick1_part_len
        max_val = gears_dis - radius1 + stick1_part_len
        if min_val < max_val:
            # print(f"will sample from {min_val} to  {max_val}")
            stick_param["length"] = round(
                random.uniform(gears_dis + radius1 + radius2 - stick1_part_len, gears_dis - radius1 + stick1_part_len),
                2)
            return stick_param

    stick_param["length"] = round(sample_length_from_current(stick_param["length"], diff_val=diff_val), 2)
    return stick_param


def sample_position(joint_location, diff_val=2, num_of_axis=3, enable_negative=True):
    for i in range(num_of_axis):
        new_pos = round(joint_location[i] + random.uniform(-diff_val, diff_val), 2)
        if not enable_negative:
            while new_pos < 0:
                new_pos = round(joint_location[i] + random.uniform(-diff_val, diff_val), 2)
        joint_location[i] = new_pos

    return joint_location


def sample_point(point, diff_val=2, num_of_axis=3):
    vector = point.vector()
    vector = sample_position(vector, diff_val=diff_val, num_of_axis=num_of_axis)
    return Point(*vector)


def sample_from_cur_assemblyA(assemblyA, gear_diff_val=0.5, stick_diff_val=0.5, position_diff_val=0.5, random_sample=1):
    config = assemblyA.config

    if random.random() < random_sample:
        config["gear1_init_parameters"] = sample_gear_parameters_from_current(config["gear1_init_parameters"],
                                                                              gear_diff_val)
    if random.random() < random_sample:
        config["gear2_init_parameters"] = sample_gear_parameters_from_current(config["gear2_init_parameters"],
                                                                              gear_diff_val,second_gear = True,\
                                                                              gear1_radius = config["gear1_init_parameters"]["radius"])
    if random.random() < random_sample:
        config["stick1_init_parameters"] = sample_stick_parameters_from_current(config["stick1_init_parameters"],
                                                                                stick_diff_val)

    if random.random() < random_sample:
        config["gear1_stick1_joint_location"] = sample_position(config["gear1_stick1_joint_location"],
                                                                position_diff_val, num_of_axis=2)

    if random.random() < random_sample:
        config["gear2_stick2_joint_location"] = sample_position(config["gear2_stick2_joint_location"],
                                                                position_diff_val, num_of_axis=2)
    if random.random() < random_sample:
        config["stick1_stick2_joint_location"] = sample_position(config["stick1_stick2_joint_location"],
                                                                 position_diff_val, num_of_axis=1,
                                                                 enable_negative=False)
    if random.random() < random_sample:
        config["stick2_stick1_joint_location"] = (config["stick2_init_parameters"]["length"], 0, 0)

    # for fixed :
    # config["gear1_fixed_position"] = sample_point(config["gear1_fixed_position"], position_diff_val, num_of_axis=1)
    # config["gear2_fixed_position"] = sample_point(config["gear2_fixed_position"], position_diff_val, num_of_axis=1)

    # for fixed2 :
    if random.random() < random_sample:
        config["gear1_fixed_position"] = sample_position(config["gear1_fixed_position"], position_diff_val,
                                                         num_of_axis=2)
    if random.random() < random_sample:
        config["gear2_fixed_position"] = sample_position(config["gear2_fixed_position"], position_diff_val,
                                                         num_of_axis=2)
    if random.random() < random_sample:
        radius1 = config["gear1_init_parameters"]["radius"]
        radius2 = config["gear2_init_parameters"]["radius"]
        gears_dis = points_distance(config["gear1_fixed_position"], config["gear2_fixed_position"])
        stick1_part_len = config["stick1_stick2_joint_location"][0]

        stick2_len_params = (radius1, radius2, gears_dis, stick1_part_len)
        config["stick2_init_parameters"] = sample_stick_parameters_from_current(config["stick2_init_parameters"],
                                                                                stick_diff_val, stick2_len_params)
    return AssemblyA(config)


def points_distance(point1, point2):
    # for fixed:
    # point1_vector = point1.vector()
    # point2_vector = point2.vector()

    # for fixed2:
    point1_vector = point1
    point2_vector = point2
    dis = 0
    for i in range(len(point1_vector)):
        dis += (point1_vector[i] - point2_vector[i]) ** 2
    return round(dis ** 0.5, 2)


def is_vaild_assembleA(assemblyA, debug_mode=False):
    config = assemblyA.config


    if config["stick1_init_parameters"]["length"] < config["stick1_stick2_joint_location"][0]:
        if debug_mode:
            print(
                f"stick length {config['stick1_init_parameters']['length']} and joint location is in {config['stick1_stick2_joint_location'][0]}")
        return False

    if config["stick2_init_parameters"]['length'] < config["stick2_stick1_joint_location"][0]:
        if debug_mode:
            print(
                f"stick length {config['stick2_init_parameters']['length']} and joint location is in {config['stick2_stick1_joint_location'][0]}")

        return False

    joint_x1, joint_y1 = config["gear1_stick1_joint_location"][:2]
    center_x1, center_y1 = (0, 0)
    radius1 = config["gear1_init_parameters"]["radius"]

    if (joint_x1 - center_x1) ** 2 + (joint_y1 - center_y1) ** 2 > radius1 ** 2:
        if debug_mode:
            print(
                f"center gear 1 {center_x1, center_y1} with radius {radius1} and joint location is in {joint_x1, joint_y1}")
        return False

    joint_x2, joint_y2 = config["gear2_stick2_joint_location"][:2]
    center_x2, center_y2 = (0, 0)
    radius2 = config["gear2_init_parameters"]["radius"]
    if (joint_x2 - center_x2) ** 2 + (joint_y2 - center_y2) ** 2 > radius2 ** 2:
        if debug_mode:
            print(
                f"center gear 2 {center_x2, center_y2} with radius {radius2} and joint location is in {joint_x2, joint_y2}")
        return False

    gears_dis = points_distance(config["gear1_fixed_position"], config["gear2_fixed_position"])
    stick2_len = config["stick2_init_parameters"]["length"]
    stick1_part_len = config["stick1_stick2_joint_location"][0]

    if (gears_dis + radius1 + radius2) >= stick2_len + stick1_part_len:
        if debug_mode:
            print(
                f"gears distance is {gears_dis} with radius {radius1, radius2} and max length between sticks is {stick2_len + stick1_part_len}")
            print("sticks too short")
        return False

    if gears_dis - radius1 + stick1_part_len < stick2_len:
        if debug_mode:
            print(
                f"gears distance is {gears_dis} with radius1 {radius1} and stick1 len is {stick1_part_len} and stick2 len {stick2_len}")
            print("sticks too long")
        return False
    # except Exception as e:
    # print(f"error = {e}")
    # return False

    return True


def get_assembly_curve(assembly, number_of_points=360, plot_path=None, save_images=False, normelaize_curve=False):
    assembly_curve = []
    actuator = assembly.actuator
    for i in range(number_of_points):
        actuator.turn(360 / number_of_points)
        result = assembly.update_state2()
        if result:
            assembly_curve.append(assembly.get_red_point_position())
            if plot_path:
                assembly.plot_assembly(plot_path=plot_path, image_number=i, save_images=save_images)
    return Curve(normalize_curve2(assembly_curve, assembly.anchor) if normelaize_curve else assembly_curve)


def get_assembly_curve_parallel(assembly, number_of_points=360):
    import multiprocessing
    from joblib import Parallel, delayed
    num_cores = multiprocessing.cpu_count()
    import time

    def f(i, orig):
        # cpy = AssemblyA(orig.config)
        orig.actuator.set(i * (360.0 / number_of_points))
        result = orig.update_state2()
        if result:
            return orig.get_red_point_position()
        return [0, 0, i * (360.0 / number_of_points)]

    # print("started")
    start = time.time()
    # x0 = assembly.update_state3()
    # assembly_curve = Parallel(n_jobs=num_cores)(delayed(f)(i, assembly) for i in range(number_of_points))
    assembly_curve = [f(i, assembly) for i in range(number_of_points)]
    end = time.time()
    # print(end-start)
    return Curve(assembly_curve)


def is_dissimilar(curve, database, gamma=1):
    for database_curve in database:
        if Curve.normA(curve, database_curve) < gamma:
            return False
    return True

def return_prototype2():
    config = dict()

    config["gear1_init_parameters"] = {"radius": 4}
    config["stick1_init_parameters"] = {"length": 16}
    config["gear2_init_parameters"] = {"radius": 2}
    config["stick2_init_parameters"] = {"length": 9}

    # config["gear1_init_parameters"] = {"radius": 2, "center": Point(0.0, 0.0, 0.0), "orientation": Alignment(0, 0, 0)}
    # config["stick1_init_parameters"] = {"length": 6, "edge": Point(0, 0, 0), "orientation": Alignment(0, 0, 0)}
    # config["gear2_init_parameters"] = {"radius": 1, "center": Point(6.0, 0.0, 0.0), "orientation": Alignment(0, 0, 0)}
    # config["stick2_init_parameters"] = {"length": 4, "edge": Point(0.0, 0.0, 0.0), "orientation": Alignment(0, 0, 0)}

    # for fixed
    # config["gear1_fixed_position"] = Point(0.0, 0.0, 0.0)
    # config["gear2_fixed_position"] = Point(6.0, 0.0, 0.0)

    # for fixed2
    config["gear1_fixed_position"] = np.array([0, 0, 0], dtype=float)
    config["gear2_fixed_position"] = np.array([11, 0, 0], dtype=float)

    # for fixed
    # config["gear1_fixed_orientation"] = Alignment(0, 0, 0)
    # config["gear2_fixed_orientation"] = Alignment(0, 0, 0)

    # for fixed2
    config["gear1_fixed_orientation"] = np.array([0, 0, 0.5 * np.pi], dtype=float)
    config["gear2_fixed_orientation"] = np.array([0, 0, 0.5 * np.pi], dtype=float)

    config["gear1_stick1_joint_location"] = np.array([4, 0, 0], dtype=float)
    config["stick1_gear1_joint_location"] = np.array([0, 0, 0], dtype=float)
    config["gear2_stick2_joint_location"] = np.array([0, 0.5, 0], dtype=float)
    config["stick2_gear2_joint_location"] = np.array([0, 0, 0], dtype=float)

    config["stick1_stick2_joint_location"] = np.array([10.5, 0, 0], dtype=float)
    config["stick2_stick1_joint_location"] = np.array([config["stick2_init_parameters"]["length"], 0, 0], dtype=float)

    return AssemblyA(config)


def return_prototype():
    config = dict()

    config["gear1_init_parameters"] = {"radius": 5}
    config["stick1_init_parameters"] = {"length": 14}
    config["gear2_init_parameters"] = {"radius": 2}
    config["stick2_init_parameters"] = {"length": 8}

    # config["gear1_init_parameters"] = {"radius": 2, "center": Point(0.0, 0.0, 0.0), "orientation": Alignment(0, 0, 0)}
    # config["stick1_init_parameters"] = {"length": 6, "edge": Point(0, 0, 0), "orientation": Alignment(0, 0, 0)}
    # config["gear2_init_parameters"] = {"radius": 1, "center": Point(6.0, 0.0, 0.0), "orientation": Alignment(0, 0, 0)}
    # config["stick2_init_parameters"] = {"length": 4, "edge": Point(0.0, 0.0, 0.0), "orientation": Alignment(0, 0, 0)}

    # for fixed
    # config["gear1_fixed_position"] = Point(0.0, 0.0, 0.0)
    # config["gear2_fixed_position"] = Point(6.0, 0.0, 0.0)

    # for fixed2
    config["gear1_fixed_position"] = np.array([0, 0, 0], dtype=float)
    config["gear2_fixed_position"] = np.array([10, 0, 0], dtype=float)

    # for fixed
    # config["gear1_fixed_orientation"] = Alignment(0, 0, 0)
    # config["gear2_fixed_orientation"] = Alignment(0, 0, 0)

    # for fixed2
    config["gear1_fixed_orientation"] = np.array([0, 0, 0.5 * np.pi], dtype=float)
    config["gear2_fixed_orientation"] = np.array([0, 0, 0.5 * np.pi], dtype=float)

    config["gear1_stick1_joint_location"] = np.array([4, 0, 0], dtype=float)
    config["stick1_gear1_joint_location"] = np.array([0, 0, 0], dtype=float)
    config["gear2_stick2_joint_location"] = np.array([1, 0, 0], dtype=float)
    config["stick2_gear2_joint_location"] = np.array([0, 0, 0], dtype=float)

    config["stick1_stick2_joint_location"] = np.array([12, 0, 0], dtype=float)
    config["stick2_stick1_joint_location"] = np.array([config["stick2_init_parameters"]["length"], 0, 0], dtype=float)

    return AssemblyA(config)


def create_assemblyA(gear_diff_val=1, stick_diff_val=1, position_diff_val=1):
    new_assembly = sample_from_cur_assemblyA(return_prototype2(), gear_diff_val=gear_diff_val,
                                             stick_diff_val=stick_diff_val, position_diff_val=position_diff_val,
                                             random_sample=0.8)
    while not is_vaild_assembleA(new_assembly):
        # print("not valid assembly")
        new_assembly = sample_from_cur_assemblyA(return_prototype2(), gear_diff_val=gear_diff_val,
                                                 stick_diff_val=stick_diff_val, position_diff_val=position_diff_val,
                                                 random_sample=0.8)
    return new_assembly


def create_random_assembly_A(gear_diff_val=1, stick_diff_val=1, position_diff_val=1):
    new_assembly = sample_from_cur_assemblyA(return_prototype2(), gear_diff_val=gear_diff_val,
                                             stick_diff_val=stick_diff_val, position_diff_val=position_diff_val,
                                             random_sample=0.8)
    while not is_vaild_assembleA(new_assembly):
        # print("not valid assembly")
        new_assembly = sample_from_cur_assemblyA(return_prototype2(), gear_diff_val=gear_diff_val,
                                                 stick_diff_val=stick_diff_val, position_diff_val=position_diff_val,
                                                 random_sample=0.8)
    return new_assembly


class AssemblyA_Sampler:
    def __init__(self, number_of_points=72, num_of_samples_around=10):
        self.database = []
        self.curve_database = []
        self.number_of_points = number_of_points
        self.num_of_samples_around = num_of_samples_around

    def recursive_sample_assemblyA(self, assemblyA, num_of_samples_around=None, debug_mode=False):
        if not num_of_samples_around:
            num_of_samples_around = self.num_of_samples_around
        accepted_assemblies = []
        for i in range(num_of_samples_around):
            new_assemblyA = sample_from_cur_assemblyA(assemblyA, random_sample=0.5, )
            if is_vaild_assembleA(new_assemblyA, debug_mode=debug_mode):
                if debug_mode:
                    print("valid assembly!")
                assembly_curve = get_assembly_curve(new_assemblyA, number_of_points=self.number_of_points,
                                                    normelaize_curve=True)
                # assembly_curve = [1]
                if is_dissimilar(assembly_curve, self.curve_database):
                    if debug_mode:
                        print(f"----------------added assembly {len(self.curve_database)}----------------")
                    self.curve_database.append(assembly_curve)
                    accepted_assemblies.append(new_assemblyA)
                elif (debug_mode):
                    print(f"assembly too similar to db")

        return accepted_assemblies

    def get_origin_assembly(self):
        origin_assembly = create_assemblyA()
        origin_curve = get_assembly_curve(origin_assembly, number_of_points=self.number_of_points,
                                          normelaize_curve=True)

        while not is_dissimilar(origin_curve, self.curve_database):
            # print("Origin assembly too similar to current assemblies")
            origin_assembly = create_assemblyA()
            origin_curve = get_assembly_curve(origin_assembly, number_of_points=self.number_of_points,
                                              normelaize_curve=True)

        self.database += [origin_assembly]
        self.curve_database += [origin_curve]

        return origin_assembly, origin_curve

    def create_assemblyA_database(self, min_samples_number=1000, num_of_samples_around=None, debug_mode=False):
        if not num_of_samples_around:
            num_of_samples_around = self.num_of_samples_around
        cur_database_len = len(self.database)

        origin_assembly, _ = self.get_origin_assembly()

        if debug_mode:
            print(f"origin_assembly initiaized")
            print(f"curve_database is {self.curve_database}")

        while len(self.database) - cur_database_len < min_samples_number:
            if debug_mode:
                print(f"current database size {len(self.database)}")
            accepted_assemblies = self.recursive_sample_assemblyA(origin_assembly,
                                                                  num_of_samples_around=num_of_samples_around,
                                                                  debug_mode=debug_mode)
            self.database += accepted_assemblies
            while len(accepted_assemblies) > 0 and len(self.database) < min_samples_number:
                origin_assembly = accepted_assemblies[0]
                neighbor_accepted_assemblies = self.recursive_sample_assemblyA(origin_assembly,
                                                                               num_of_samples_around=num_of_samples_around,
                                                                               debug_mode=debug_mode)
                self.database += neighbor_accepted_assemblies
                accepted_assemblies = accepted_assemblies[1:]
                accepted_assemblies += neighbor_accepted_assemblies
            if debug_mode:
                print("---we will get another origin assembly---")
            origin_assembly, _ = self.get_origin_assembly()

    def get_database(self):
        return self.database

    def get_curve_database(self):
        return self.curve_database

    def get_closest_curve(self, curve, get_all_dis = False):

        min_dis = curve.normA(curve, self.curve_database[0])
        min_curve = self.curve_database[0]
        closest_assembly = self.database[0]
        if get_all_dis:
            all_dist = {}
        for i,db_curve in enumerate(self.curve_database[1:]):
            cur_dis = curve.normA(curve, db_curve)
            if get_all_dis:
                all_dist[db_curve] = cur_dis
            if cur_dis < min_dis:
                closest_assembly = self.database[i+1]
                min_dis = cur_dis
                min_curve = db_curve
        # return min_curve.to_json(s)
        return min_curve,closest_assembly,all_dist if get_all_dis else None

    def save(self, path=r"C:\Users\A\Desktop\temp"):
        with open(path + rf"\sampler", "wb") as handle:
            pickle.dump(self, handle)

    @staticmethod
    def load(path=r"C:\Users\A\Desktop\temp"):
        with open(path, "rb") as handle:
            sampler = pickle.load(handle)
        return sampler


def normalize_curve(curve, anchor):
    return ([list(sample - anchor) for sample in curve])



def normalize_curve2(curve_points):
    x_com = np.mean(curve_points, axis=0)
    centered = curve_points - x_com
    pca = PCA(n_components=2)
    pca.fit(centered)
    v_max = pca.components_[0]
    l_max = pca.explained_variance_[0]
    projected_points = pca.transform(centered)
    zeros = np.zeros((len(projected_points),1),dtype=np.float64)
    np.concatenate([projected_points,zeros],axis=1)
    return np.concatenate([projected_points,zeros],axis=1)/l_max

def plot_circle(ax, x, y, r):
    theta = np.linspace(0, 2 * np.pi, 100)

    x1 = r * np.cos(theta) + x
    x2 = r * np.sin(theta) + y

    ax.plot(x1, x2)
    ax.set_aspect(1)


class AssemblyA(Assembly):

    def __init__(self, config):
        self.config = config
        self._parse_config(self.config)

    def _parse_config(self, config):

        self.actuator = Actuator()
        self.components = [Gear(**config["gear1_init_parameters"]), Stick(**config["stick1_init_parameters"]),
                           Gear(**config["gear2_init_parameters"]), Stick(**config["stick2_init_parameters"]), Gear(1),
                           Gear(1)]
        alignment = np.array([0, 0, 0.5 * np.pi])

        self.connections = [PhaseConnection2(self.components[0], self.actuator),
                            PhaseConnection2(self.components[0], self.components[2]),

                            FixedConnection2(self.components[4], config["gear1_fixed_position"],
                                             config["gear1_fixed_orientation"]),
                            FixedConnection2(self.components[5], config["gear2_fixed_position"],
                                             config["gear2_fixed_orientation"]),

                            PinConnection2(self.components[0], self.components[4], np.array([0, 0, 0]),
                                           np.array([0, 0, 0]),
                                           alignment, alignment),
                            PinConnection2(self.components[2], self.components[5], np.array([0, 0, 0]),
                                           np.array([0, 0, 0]), alignment, alignment),

                            PinConnection2(self.components[0], self.components[1],
                                           config["gear1_stick1_joint_location"],
                                           config["stick1_gear1_joint_location"], alignment, alignment),
                            PinConnection2(self.components[2], self.components[3],
                                           config["gear2_stick2_joint_location"],
                                           config["stick2_gear2_joint_location"], alignment, alignment),
                            PinConnection2(self.components[1], self.components[3],
                                           config["stick1_stick2_joint_location"],
                                           config["stick2_stick1_joint_location"], alignment, alignment)]

        self.red_point_component = self.components[1]
        anchor = []
        # for fixed:
        # for axis_1,axis_2 in zip(config["gear1_fixed_position"].vector(),config["gear2_fixed_position"].vector()):

        # for fixed2:
        for axis_1, axis_2 in zip(config["gear1_fixed_position"], config["gear2_fixed_position"]):
            anchor.append(round((axis_1 + axis_2) / 2, 2))
        self.anchor = np.array(anchor)
        Assembly.__init__(self, self.connections, self.components, self.actuator)

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

    def translate_assembly(self, point):
        for con in self.con_list:
            if isinstance(con, FixedConnection2):
                con.fixed_position = con.fixed_position + point


class StickFigure(Assembly):

    def __init__(self):
        rleg = Stick(20)
        lleg = Stick(20)
        body = Stick(40)
        rhand1 = Stick(7)
        rhand2 = Stick(7)
        rhand3 = Stick(3)
        lhand1 = Stick(7)
        lhand2 = Stick(7)
        lhand3 = Stick(3)
        head = Gear(5)
        comp_lst = [rleg,
                    lleg,
                    body,
                    rhand1,
                    rhand2,
                    # rhand3,
                    lhand1,
                    lhand2,
                    # lhand3,
                    head
                    ]
        self.comp_dict = {'rleg': rleg,
                          'lleg': lleg,
                          'body': body,
                          'rhand1': rhand1,
                          'rhand2': rhand2,
                          # 'rhand3': rhand3,
                          'lhand1': lhand1,
                          'lhand2': lhand2,
                          # 'lhand3': lhand3,
                          'head': head}
        crotch_x, crotch_y = 15, 15 - 30
        con_lst = [
            PinConnection2(body, rleg, Point(0, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0), Alignment(0, 0, 0)),
            PinConnection2(body, lleg, Point(0, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0), Alignment(0, 0, 0)),
            PinConnection2(body, rhand1, Point(body.length - 2 * head.radius, 0, 0),
                           Point(0, 0, 0), Alignment(0, 0, 0), Alignment(0, 0, 0)),
            PinConnection2(rhand1, rhand2, Point(rhand1.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            # PinConnection2(rhand2, rhand3, Point(rhand2.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
            #                Alignment(0, 0, 0)),
            PinConnection2(body, lhand1, Point(body.length - 2 * head.radius, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            PinConnection2(lhand1, lhand2, Point(lhand1.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            # PinConnection2(lhand2, lhand3, Point(lhand2.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
            #                Alignment(0, 0, 0)),
            PinConnection2(body, head, Point(body.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            FixedConnection2(lleg, Point(crotch_x, crotch_y, 0), Alignment(0, 0, -135)),
            FixedConnection2(rleg, Point(crotch_x, crotch_y, 0), Alignment(0, 0, -45)),
            FixedConnection2(head, Point(crotch_x, crotch_y + body.length, 0), Alignment(0, 0, 0)),
        ]
        Assembly.__init__(self, con_lst, comp_lst)

    def add_driving_assembly(self, driving_mec):
        combined_asm = self.merge_assembly(driving_mec)
        redp_comp = driving_mec.red_point_component
        combined_asm.con_list = combined_asm.con_list + [PinConnection2(self.comp_dict['rhand2'],
                                                                        redp_comp,
                                                                        Point(self.comp_dict['rhand2'].length, 0, 0),
                                                                        Point(redp_comp.length, 0, 0),
                                                                        Alignment(0, 0, 0), Alignment(0, 0, 0))]
        return combined_asm


class StickSnake(Assembly):

    def __init__(self):
        stick1 = Stick(7)
        stick2 = Stick(7)
        stick3 = Stick(7)
        stick4 = Stick(7)
        stick5 = Stick(7)
        origin = Gear(1)
        comp_lst = [stick1,
                    stick2,
                    stick3,
                    stick4,
                    stick5,
                    origin,
                    ]
        self.comp_dict = {'stick1': stick1,
                          'stick2': stick2,
                          'stick3': stick3,
                          'stick4': stick4,
                          'stick5': stick5,
                          'origin': origin,
                          }
        fix_x, fix_y = 15, 15
        con_lst = [
            PinConnection2(stick1, stick2, Point(stick1.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            PinConnection2(stick2, stick3, Point(stick2.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            PinConnection2(stick3, stick4, Point(stick3.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            PinConnection2(stick4, stick5, Point(stick4.length, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            PinConnection2(stick1, origin, Point(0, 0, 0), Point(0, 0, 0), Alignment(0, 0, 0),
                           Alignment(0, 0, 0)),
            FixedConnection2(origin, Point(fix_x, fix_y, 0), Alignment(0, 0, 0)),
        ]
        Assembly.__init__(self, con_lst, comp_lst)

    def add_driving_assembly(self, driving_mec):
        combined_asm = self.merge_assembly(driving_mec)
        redp_comp = driving_mec.red_point_component
        combined_asm.con_list = combined_asm.con_list + [PinConnection2(self.comp_dict['stick5'],
                                                                        redp_comp,
                                                                        Point(self.comp_dict['stick5'].length, 0, 0),
                                                                        Point(redp_comp.length, 0, 0),
                                                                        Alignment(0, 0, 0),
                                                                        Alignment(0, 0, 0))]
        return combined_asm

# origin_assembly = create_assemblyA()
# print(is_vaild_assembleA(origin_assembly,debug_mode=True))
# curve = get_assembly_curve(origin_assembly,plot_path=r"C:\Users\A\Desktop\temp",number_of_points=72,normelaize_curve=False,save_images=True)
# curve.plot(save_image=True)

# database, curve_database = create_assemblyA_database(2,2)
#
# print(len(database))
# print(len(curve_database))
#
# config = create_assemblyA().config
#
# config = database[0].config
# print("------------")
# for key1 in config:
#     print(key1)
#     if isinstance(config[key1], dict):
#         for key in config[key1]:
#             print(key)
#             print(config[key1][key])
#     else:
#         print(config[key1])
#


# assembly = return_prototype()
# actuator = assembly.actuator
#
#
# gear1 = assembly.components[0]
# gear2 = assembly.components[2]
# stick1 = assembly.components[1]
# stick2 = assembly.components[3]
#
# for i in range(100):
#     t = time.time()
#     actuator.turn(1)
#     print("success: ", assembly.update_state())
#     print("time: ", time.time() - t)
#
#     print('actuator turned: ', i)
#     print('gear1 alpha:', np.rad2deg(gear1.configuration.alignment.vector()))
#     print('gear1 position:', gear1.configuration.position.vector())
#     print('gear2 orientation:', np.rad2deg(gear2.configuration.alignment.vector()))
#     print('gear2 position:', gear2.configuration.position.vector())
#     print('stick1 orientation:', np.rad2deg(stick1.configuration.alignment.vector()))
#     print('stick1 position:', stick1.configuration.position.vector())
#     print('red point position:', assembly.get_red_point_position())
#     print("***************************************************************")
