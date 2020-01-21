from assembly import *
from sampler import AssemblyA_Sampler
import argparse
from os.path import join as pjoin

parser = argparse.ArgumentParser()

parser.add_argument('n', metavar='n', type=int,
                    help='number of desired samples')
parser.add_argument('--file_path', metavar='p', type=str, default=pjoin("dbs", "new_db"),
                    help='sampler destination path')
parser.add_argument('--load_db', metavar='l', type=bool, default=False,
                    help='load current sampler from specified path, new samples will be added to it')
parser.add_argument('--debug_mode', metavar='d', type=bool, default=False,
                    help='debug mode')


def main(n, file_path=pjoin("dbs", "new_db"), load_db=False, debug_mode=False):
    print(n)
    if load_db:
        sample = AssemblyA_Sampler.load(file_path)
    else:
        sample = AssemblyA_Sampler()

    sample.create_assemblyA_database((3 * n) // 4, num_of_samples_around=10, debug_mode=debug_mode, second_type=False)
    sample.create_assemblyA_database(n // 4, num_of_samples_around=10, debug_mode=debug_mode, second_type=True)
    sample.save(file_path)


if __name__ == "__main__":
    args = parser.parse_args()
    main(**vars(args))
