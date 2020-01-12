from assembly import *
from connections2 import *
import sys
import json

def normalize_curve(curve,anchor):
    return ([list(sample-anchor) for sample in curve ])

def generate_random_curve(number_of_curves , number_of_points=360 , curve_path = r'C:\Users\A\Desktop\temp'):

    # print(f"will generate {number_of_curves} curves with {number_of_points} points")
    # if not os.path.exists(curve_path):
    #     os.makedirs(curve_path)
    components = [Gear(5), Stick(10),Gear(2), Stick(5)]

    connections = [     PhaseConnection(components[0],Actuator()),
                        FixedConnection2(components[0], np.array([0,0,0]),np.array([0,0,0])),
                        FixedConnection2(components[2], np.array([10,0,0]),np.array([0,0,0])),
                        PinConnection2(components[0], components[1],np.array([4,0,0]),np.array([1,0,0]),Alignment(0,0,1),Alignment(0,0,1)),
                        PinConnection2(components[2], components[3],np.array([1,0,0]),np.array([0,0,0]),Alignment(0,0,1),Alignment(0,0,1)),
                        PinConnection2(components[1], components[3],np.array([7,0,0]),np.array([5,0,0]),Alignment(0,0,1),Alignment(0,0,1))]
    origin_assembly = Assembly(connections,components)

    for i in range (int(number_of_curves)):
        # origin_assembly = create_assemblyA()
        assembly_curve = get_assembly_curve(origin_assembly, number_of_points=number_of_points)
        assembly_curve = normalize_curve(assembly_curve, origin_assembly.anchor)
        json.dump(assembly_curve, open(curve_path + rf"\curve_{i}.j", "w"))

if __name__ == "__main__":
    generate_random_curve(*sys.argv[1:])