from assembly import *
from curve import Curve


def normalize_curve(curve, anchor):
    return ([list(sample - anchor) for sample in curve.points])


def generate_random_curve(number_of_points=360, gear_diff_val=1, stick_diff_val=1, position_diff_val=1):
    random_assembly_a = create_assemblyA(gear_diff_val=gear_diff_val, stick_diff_val=stick_diff_val, \
                                         position_diff_val=position_diff_val)
    assembly_curve = get_assembly_curve_parallel(random_assembly_a, number_of_points=number_of_points)

    assembly_curve = normalize_curve2(assembly_curve, random_assembly_a.anchor)

    c = Curve(assembly_curve)
    # output curve to stdout for the C# to read
    print(c.to_json())


if __name__ == "__main__":
    generate_random_curve(number_of_points=76, gear_diff_val=1, stick_diff_val=2, position_diff_val=1)
