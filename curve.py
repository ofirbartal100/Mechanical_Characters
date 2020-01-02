import numpy as np
import numpy.linalg as alg


class Curve:
    '''
    this class represents a curve by its points and its features
    '''

    def __init__(self, normalized_coordinates):
        self.features = np.zeros((6,), dtype=np.float64)
        self.points = normalized_coordinates
        self._calculate_features(self)

    def _calculate_features(self):
        np = len(self.points)
        e = np.zeros(self.points.shape, dtype=np.float64)
        for i in range(np):
            e[i] = self.points[i] - self.points[(i - 1 + np) % np]

        t = np.zeros(self.points.shape, dtype=np.float64)
        for i in range(np):
            t[i] = e[i] / alg.norm(e[i])

        x_com = np.mean(self.points)

        # f0
        self.features[0] = np.sum(np.array([alg.norm(_e) for _e in e]))

        f1 = 0
        for i in range(np):
            f1 += np.cross(self.points[i], self.points[(i - 1 + np) % np])
        self.features[1] = f1

        # f2 - calculate l_min/l_max using PCA
        v_max = 0
        v_min = 0
        l_max = 0
        l_min = 0

        # f3
        self.features[3] = alg.norm(x_com)

        # f4
        self.features[4] = np.arcsin(alg.norm(np.cross(x_com / alg.norm(x_com), v_max)))


        # f5 - count intersections
