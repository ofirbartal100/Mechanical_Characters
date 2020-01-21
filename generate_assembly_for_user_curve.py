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


parser = argparse.ArgumentParser(
    description='Gets a user defined curve as a points list and searching the DB for closest representing curve')
parser.add_argument('-json_path', help='a path to the file containing a json formatted points list')
parser.add_argument('-db_path', help='a path to the db file')

if __name__ == "__main__":
    # arguments parser
    args = parser.parse_args()

    # handle input
    curve = read_input_as_curve(args.json_path)

    input_file = open(args.db_path, 'rb')
    sample = dill.load(input_file)

    db_closest_curve, assembly, db_curve = sample.get_closest_curve(curve, get_all_dis=True)

    c = {}
    c['curve'] = {'points': db_closest_curve.points.tolist(), 'features': db_closest_curve.features.tolist()}
    c['assembly'] = {'components': assembly.describe_assembly()}
    print(json.dumps(c))
