from assembly import *
import json
import argparse

from curve import Curve


def read_input_as_curve(json_path):
    # read file
    with open(json_path, 'r') as j:
        data = j.read()
    # parse file
    points = json.loads(data)
    return Curve(points)

parser = argparse.ArgumentParser(description='Gets a user defined curve as a points list and searching the DB for closest representing curve')
parser.add_argument('-json_path', help='a path to the file containing a json formatted points list')

if __name__ == "__main__":
    # arguments parser
    args = parser.parse_args()


    # handle input
    curve = read_input_as_curve(args.json_path)

    # # for some reason it is instance based and not static ?
    # assembly_A_sampler = AssemblyA_Sampler(number_of_points=76)
    #
    # closest_curve = assembly_A_sampler.get_closest_curve(curve)

    # output to standard output
    # print(closest_curve.to_json())
    print(curve.to_json())
