from parts_classes import *
from scipy import optimize
import numpy as np

gear_conf = Configuration(Point(0, 0, 0), Alignment(0, 0, 0))
stick_conf = Configuration(Point(1, 1, 1), Alignment(0, 0, 0))
gear = Gear(gear_conf, 1)
stick = Stick(stick_conf, 1)


def master_constraint(params):
    for const, param_pair in const_list:
        sum += const(param_pair)
    return sum

sol = optimize.newton(master_constraint, np.random.randint(0,10,2), maxiter=1000)
print(constraint(sol), sol)
