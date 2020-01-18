from assembly import *
import json
import argparse
import dill

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

    input_file = open(r"C:\Users\ofir\Desktop\sampler", 'rb')
    sample = dill.load(input_file)

    target = create_assemblyA()
    # print(is_vaild_assembleA(target))
    target_curve = get_assembly_curve(target, number_of_points=72)
    db_closest_curve, all_dist = sample.get_closest_curve(curve, get_all_dis=True)
    # for k in all_dist:
    #     print(all_dist[k])
    # print("-------")
    # print(all_dist[db_closest_curve])

    # output to standard output
    # print(closest_curve.to_json())
    print(db_closest_curve.to_json())
