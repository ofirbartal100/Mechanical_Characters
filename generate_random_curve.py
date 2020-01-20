from assembly import *
from connections2 import *

import json
import os
import pickle


def normalize_curve(curve, anchor):
    return ([list(sample - anchor) for sample in curve])


def generate_random_curve(number_of_curves, number_of_points=360, curve_path=None, \
                          gear_diff_val=1, stick_diff_val=1, position_diff_val=1, save_images=False):
    print(f"will generate {number_of_curves} curves with {number_of_points} points")
    if not os.path.exists(curve_path):
        os.makedirs(curve_path)

    for i in range(number_of_curves):
        if curve_path:
            plot_path = curve_path + fr"\curve_{i}"
            if not os.path.exists(plot_path):
                os.mkdir(plot_path)
        else:
            plot_path = curve_path

        print(f"generate curve number {i}")
        origin_assembly = create_assemblyA(gear_diff_val=gear_diff_val, stick_diff_val=stick_diff_val, \
                                           position_diff_val=position_diff_val)

        assembly_curve = get_assembly_curve(origin_assembly, number_of_points=number_of_points, \
                                            plot_path=plot_path, save_images=save_images)
        assembly_curve = normalize_curve2(assembly_curve, origin_assembly.anchor)

        json.dump(assembly_curve, open(curve_path + rf"\curve_{i}.j", "w"))

        with open(curve_path + rf"\assembly_{i}_description.p", "wb") as handle:
            pickle.dump(origin_assembly.config, handle, protocol=pickle.HIGHEST_PROTOCOL)


def generate_random_curve2(number_of_curves, number_of_points=360, curve_path=r'C:\Users\A\Desktop\temp'):
    # print(f"will generate {number_of_curves} curves with {number_of_points} points")
    # if not os.path.exists(curve_path):
    #     os.makedirs(curve_path)
    components = [Gear(5), Stick(10), Gear(2), Stick(5), Gear(1)]
    actuator = Actuator()
    alignment = np.array([0, 0, 0.5 * np.pi])
    connections = [PhaseConnection2(components[0], actuator),
                   FixedConnection2(components[4], np.array([0, 0, 0]), alignment),
                   FixedConnection2(components[2], np.array([10, 0, 0]), alignment),

                   PinConnection2(components[0], components[4], np.array([0, 0, 0]), np.array([0, 0, 0]), alignment,
                                  alignment),
                   PinConnection2(components[0], components[1], np.array([4, 0, 0]), np.array([1, 0, 0]), alignment,
                                  alignment),
                   PinConnection2(components[2], components[3], np.array([1, 0, 0]), np.array([1, 0, 0]), alignment,
                                  alignment),
                   PinConnection2(components[1], components[3], np.array([9, 0, 0]), np.array([4, 0, 0]), alignment,
                                  alignment)]

    origin_assembly = Assembly(connections, components, actuator)

    for i in range(int(number_of_curves)):
        # origin_assembly = create_assemblyA()
        assembly_curve = get_assembly_curve(origin_assembly, number_of_points=360)
        assembly_curve = normalize_curve2(assembly_curve, origin_assembly.anchor)
        json.dump(assembly_curve, open(curve_path + rf"\curve_{i}.j", "w"))


# if __name__ == "__main__":
#     generate_random_curve(*sys.argv[1:])


generate_random_curve(1, number_of_points=10, curve_path=r'C:\Users\ofir\Desktop\curve_temp\outputs', \
                      gear_diff_val=1, stick_diff_val=2, position_diff_val=1, save_images=True)

# curve_path=r'C:\Users\A\Desktop\temp'
# origin_assembly = return_prototype()
# #print(is_vaild_assembleA(origin_assembly))
# # origin_assembly = create_assemblyA()
# assembly_curve = get_assembly_curve(origin_assembly, number_of_points=360)
# assembly_curve = normalize_curve(assembly_curve, origin_assembly.anchor)
# json.dump(assembly_curve, open(curve_path + rf"\curve_{i}.j", "w"))
