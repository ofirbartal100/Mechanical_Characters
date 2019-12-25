from connections import *
from parts_classes import *
from configuration import *
from scipy import optimize
import numpy as np

actuator_conf = Configuration(Point(0, 0, 0), Alignment(0, 0, 0))
gear1_conf = Configuration(Point(0, 0, 0), Alignment(0, 0, 0))
gear2_conf = Configuration(Point(2, 2, 0), Alignment(0, 0, 0))
gear3_conf = Configuration(Point(4, 4, 0), Alignment(0, 0, 0))

# generate gears
actuator = Gear(actuator_conf, 0)
gear1 = Gear(gear1_conf, 1)
gear2 = Gear(gear2_conf, 0.5)
gear3 = Gear(gear3_conf, 1)
stick = Stick(gear3_conf, 1)

# merge constraints to an assembly
assembly = [PhaseConnection(actuator, gear1),
            PhaseConnection(gear1, gear2),
            PhaseConnection(gear2, gear3)]
assembly_constraint, param_index = Connection.join_constraints(assembly)

# assembly_constraint_prime = Connection.join_constraints_prime(assembly)
free_params = Connection.free_params_in_assembly(assembly)
free_params_cnt = Connection.free_params_cnt_in_assembly(assembly)
# move!
actuator_conf.rotate_alpha(0.1)

# first_guess = np.random.random(free_params_cnt)
first_guess = free_params
first_guess = np.array([first_guess[k] for k in param_index])

print(assembly_constraint(first_guess))
# print(assembly_constraint_prime(np.array([])))
sol = optimize.newton(assembly_constraint, x0=first_guess, maxiter=1000)
print(sol)

# for i in range(1000):
#     print(i)
#     actuator_conf.rotate_alpha(0.00001)
#     # print(assembly_constraint(sol), sol)
#     sol = optimize.newton(assembly_constraint, sol, maxiter=1000)
