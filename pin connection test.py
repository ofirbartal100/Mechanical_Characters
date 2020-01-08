from connections import *
from parts import *
from configuration import *
from assembly import Assembly

actuator = Actuator()
gear1 = Gear(radius=1, center=Point(0, 0, 0), orientation=Alignment(0, 0, 0))
stick1 = Stick(length=1, edge=Point(1, 0, 0), orientation=Alignment(0, 0, 0))
assembly = Assembly([PhaseConnection(actuator, gear1),
                     FixedConnection(gear1, Point(0, 0, 0), Alignment(0, 0, None)),
                     PinConnection(gear1, stick1, Point(0.5, 0, 0), Point(0, 0, 0))
                     ])


actuator.turn(90)
assembly.update_state()
print('gear1 orientation:', gear1.configuration.alignment.vector())
print('gear1 position:', gear1.configuration.position.vector())
print('stick1 orientation:', stick1.configuration.alignment.vector())
print('stick1 position:', stick1.configuration.position.vector())
