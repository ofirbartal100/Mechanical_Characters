from connections import *
from parts import *
from configuration import *
from assembly import Assembly
import time

actuator = Actuator()
gear1 = Gear(radius=10)
gear2 = Gear(radius=5)
stick1 = Stick(length=20)
stick2 = Stick(length=10)

t = time.time()
assembly = Assembly([PhaseConnection(actuator, gear1),
                     PhaseConnection(gear1, gear2),
                     FixedConnection(gear1, Point(0, 0, 0), Alignment(0, 0, None)),
                     FixedConnection(gear1, Point(25, 0, 0), Alignment(0, 0, None)),
                     PinConnection(gear1, stick1, Point(10, 0, 0), Point(0, 0, 0)),
                     PinConnection(gear2, stick2, Point(5, 0, 0), Point(0, 0, 0)),
                     PinConnection(stick1, stick2, Point(10, 0, 0), Point(10, 0, 0)),
                     ])
print("assemply initialized in: ", time.time()-t)
print('gear1 alpha:', np.deg2rad(gear1.configuration.alignment.alpha))
print('gear2 alpha:', np.deg2rad(gear2.configuration.alignment.alpha))
print('stick1 orientation:', stick1.configuration.alignment.vector())
print('stick1 position:', stick1.configuration.position.vector())
print('stick2 orientation:', stick1.configuration.alignment.vector())
print('stick2 position:', stick1.configuration.position.vector())
for i in range(360):
    actuator.turn(1)
    t = time.time()
    print("success: ", assembly.update_state())

    # print('actuator turned: ', i)
    # print('gear1 orientation:', gear1.configuration.alignment.vector())
    # print('gear1 position:', gear1.configuration.position.vector())
    # print('gear2 orientation:', gear2.configuration.alignment.vector())
    # print('gear2 position:', gear2.configuration.position.vector())
    # print('stick1 orientation:', stick1.configuration.alignment.vector())
    # print('stick1 position:', stick1.configuration.position.vector())
    print("time: ", time.time()-t)
    print("***************************************************************")


