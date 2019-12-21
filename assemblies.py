from parts_classes import *
from connections import *


class AssemblyA():

    def __init__(self, config):
        self.config = config
        self._parse_config(self.config)

    def _parse_config(self, config):
        self.components = [Gear(), Stick(),Gear(), Stick()]
        self.connections = [PinConnection(self.components[0], self.components[1], 0.5, (0, 0)),
                            PinConnection(self.components[2], self.components[3], 0.5, (0, 0)),
                            PinConnection(self.components[1], self.components[3], (1, 1), (0.5, 0.5))]

    def get_constraints(self):
        C = lambda s: 0
        for connection in self.connections:
            C += connection.get_constraint()
        return C
