import json

from connections import *
from parts import *
from configuration import *
from assembly import Assembly
import time

actuator = Actuator()
gear1 = Gear(radius=4)
gear2 = Gear(radius=4)
stick1 = Stick(length=10)
stick2 = Stick(length=10)
# 1 1 5 5 6(5) 6(5) 6(5)
t = time.time()
# assembly = Assembly([PhaseConnection(actuator, gear1),
#                      PhaseConnection(gear1, gear2, phase_diff=30),
#                      FixedConnection(gear1, Point(0, 0, 0), Alignment(0, 0, None)),
#                      FixedConnection(gear2, Point(-2, 20, 0), Alignment(0, 0, None)),
#                      PinConnection(gear1, stick1, Point(4, 0, 0), Point(0, 0, 0)),
#                      PinConnection(gear2, stick2, Point(4, 0, 0), Point(0, 0, 0)),
#                      PinConnection(stick1, stick2, Point(10, 0, 0), Point(10, 0, 0)),
#                      ], components=[gear1, gear2, stick1, stick2])

assembly = Assembly([PhaseConnection(actuator, gear1),
                     # PhaseConnection(gear1, gear2),
                     FixedConnection(gear1, Point(0, 0, 0), Alignment(0, 0, None)),
                     FixedConnection(gear2, Point(0, 10, 0), Alignment(0, 0, None)),
                     PinConnection(gear1, stick1, Point(4, 0, 0), Point(0, 0, 0)),
                     PinConnection(gear2, stick1, Point(4, 0, 0), Point(10, 0, 0)),
                     ], components=[gear1, gear2, stick1], plot_newt=False, tol=3, iters=1000)


print("assemply initialized in: ", time.time()-t)
print('gear1 alpha:', np.rad2deg(gear1.configuration.alignment.alpha))
print('gear2 alpha:', np.rad2deg(gear2.configuration.alignment.alpha))
print('stick1 orientation:', stick1.configuration.alignment.vector())
print('stick1 position:', stick1.configuration.position.vector())
print('stick2 orientation:', stick2.configuration.alignment.vector())
print('stick2 position:', stick2.configuration.position.vector())
assembly.plot_assembly()
desc = []

# desc.append(assembly.describe_assembly())

for i in range(5):
    actuator.turn(45)
    print('gear1 alpha:', np.rad2deg(gear1.configuration.alignment.alpha))
    print('gear2 alpha:', np.rad2deg(gear2.configuration.alignment.alpha))
    print("success: ", assembly.update_state())
    assembly.plot_assembly()

#     t = time.time()
#     desc.append(assembly.describe_assembly())


    # print('actuator turned: ', i)
    # print('gear1 orientation:', gear1.configuration.alignment.vector())
    # print('gear1 position:', gear1.configuration.position.vector())
    # print('gear2 orientation:', gear2.configuration.alignment.vector())
    # print('gear2 position:', gear2.configuration.position.vector())
    # print('stick1 orientation:', stick1.configuration.alignment.vector())
    # print('stick1 position:', stick1.configuration.position.vector())
    # print("time: ", time.time()-t)
    # print("***************************************************************")

#TODO: generate json describing motion of assembly


