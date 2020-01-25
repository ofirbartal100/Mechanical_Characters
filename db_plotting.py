import argparse
from sampler import *

parser = argparse.ArgumentParser()

parser.add_argument('db_path', metavar='db', type=str,
                    help='sampler (db) destination path')
parser.add_argument('--curve_idx', metavar='i', type=int,
                    help='''
                    index of the desired curve to plot from the db
                    if not specified than all the curves are saved to '/db' folder''')


def main(db_path, curve_idx=None):
    with open(db_path, 'rb') as input_file:
        db_sampler = dill.load(input_file)
    if curve_idx is None:
        db_sampler.plot_all_db()
    else:
        db_sampler.plot_with_figure(curve_idx)


if __name__ == '__main__':
    args = parser.parse_args()
    main(**vars(args))
