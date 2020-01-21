from sampler import *
import sys, getopt
import argparse

def main(n,  destination_path = r"C:\Users\A\Desktop\temp", load_path = None, debug_mode = False):
    print(n)
    if load_path:
        input_file = open(n, 'rb')
        sample = dill.load(input_file)
    else:
        sample = AssemblyA_Sampler()

    sample.create_assemblyA_database((3*n)//4, num_of_samples_around=10, debug_mode=debug_mode, second_type=False)
    sample.create_assemblyA_database(n//4, num_of_samples_around=10, debug_mode=debug_mode, second_type=True)
    sample.save(destination_path)

parser = argparse.ArgumentParser()

parser.add_argument('-n', metavar='n', type=int,
                    help='number of desired samples')
parser.add_argument('--destination_path', metavar='dp', type=str, default=r"C:\Users\A\Desktop\temp",
                    help='sampler destination path')
parser.add_argument('--load_path', metavar='lp', type=str,
                    help='load cuurent sampler from this path')
parser.add_argument('--debug_mode', metavar='d', type=bool, default=False,
                    help='load cuurent sampler from this path')

if __name__ == "__main__":
    args = parser.parse_args()
    main(**vars(args))


