from connections import *
from parts import *
from configuration import *
from assembly import Assembly

# generate gears
actuator = Actuator()
gear1 = Gear(radius=1, center=Point(0, 0, 0), alignment=Alignment(0, 0, 0))
gear2 = Gear(radius=0.5, center=Point(0, 0, 0), alignment=Alignment(0, 0, 0))
gear3 = Gear(radius=0.25, center=Point(0, 0, 0), alignment=Alignment(0, 0, 0))

# merge constraints to an assembly
assembly = Assembly([PhaseConnection(actuator, gear1),
                     PhaseConnection(gear1, gear2),
                     PhaseConnection(gear2, gear3)])
# move!
for i in range(360):
    actuator.turn(1)
    assembly.update_state()
    print(assembly.cur_state)

print(gear1.configuration.alignment.alpha)
