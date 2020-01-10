
from assembly import *
import sys


def normalize_curve(curve,anchor):
    return ([list(sample-anchor) for sample in curve ])

def generate_random_curve(number_of_curves , number_of_points=360 , curve_path = r'C:\Users\A\Desktop\temp'):

    print(f"will generate {number_of_curves} curves with {number_of_points} points")
    if not os.path.exists(curve_path):
        os.makedirs(curve_path)

    for i in range (number_of_curves):
        origin_assembly = create_assemblyA()
        assembly_curve = get_assembly_curve(origin_assembly, number_of_points=number_of_points)
        assembly_curve = normalize_curve(assembly_curve, origin_assembly.anchor)
        json.dump(assembly_curve, open(curve_path + rf"\curve_{i}.j", "w"))

if __name__ == "__main__":
    generate_random_curve(*sys.argv[1:])