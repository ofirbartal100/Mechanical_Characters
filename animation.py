from matplotlib import pyplot as plt
from shapely.geometry import Polygon
from assembly import AssemblyA, Stick, Gear
from configuration import Point

polygon1 = Polygon([(0, 5),
                    (1, 1),
                    (3, 0),
                    ])
x, y = polygon1.exterior.xy
plt.plot(x, y)
plt.show()
