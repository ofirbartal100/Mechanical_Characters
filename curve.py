import numpy as np
import numpy.linalg as alg
import poly_point_isect
from sklearn.decomposition import PCA
from matplotlib import pyplot as plt
from numpy import random
import json

class Curve:
    '''
    this class represents a curve by its points and its features
    '''
    A = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

    def __init__(self, normalized_coordinates):
        self.features = np.zeros((6,), dtype=np.float64)

        if not isinstance(normalized_coordinates, np.ndarray):
            self.points = np.array(normalized_coordinates, dtype=np.float64)
        else:
            self.points = normalized_coordinates

        self._calculate_features()

    # def plot_curve(self):
    #     curve = Polygon(self.points)
    #     x, y = curve.exterior.xy
    #     plt.plot(x, y)
    #     plt.show()

    def _calculate_features(self):
        n = len(self.points)
        self._e = np.zeros(self.points.shape, dtype=np.float64)
        for i in range(n):
            self._e[i] = self.points[i] - self.points[(i - 1 + n) % n]

        self._t = np.zeros(self.points.shape, dtype=np.float64)
        for i in range(n):
            self._t[i] = self._e[i] / alg.norm(self._e[i])

        x_com = np.mean(self.points, axis=0)

        # f0 - length of curve
        self.features[0] = np.sum(np.array([alg.norm(_e) for _e in self._e]))

        # f1 - surface area between anchor and curve
        f1 = 0
        for i in range(n):
            f1 += np.linalg.norm(np.cross(self.points[i], self.points[(i - 1 + n) % n]))
        self.features[1] = f1

        # f2 - calculate l_min/l_max using PCA
        pca = PCA(n_components=2)
        pca.fit(self.points)
        v_max = pca.components_[0]
        l_max = pca.explained_variance_[0]
        l_min = pca.explained_variance_[1]
        self.features[2] = l_min / l_max

        # f3 - distance from anchor
        self.features[3] = alg.norm(x_com)

        # f4 - calculate some alignment feature
        self.features[4] = np.arcsin(alg.norm(np.cross(x_com / alg.norm(x_com), v_max)))

        # f5 - count intersections in the projected 2d curve
        projected_points = pca.transform(self.points)
        self.features[5] = len(poly_point_isect.isect_polygon(projected_points))

    def _calculate_curvature(self):
        self._curvature = np.zeros(self.points.shape)
        num_p = len(self.points)
        for i in range(num_p):
            tj_1 = self._t[(i - 1 + num_p) % num_p]
            tj = self._t[i]
            cross = np.cross(tj_1, tj)
            self._curvature[i] = ((2 / (1 + np.dot(tj, tj_1))) * cross)

    @staticmethod
    def normA(curve1, curve2):
        really_big_number = 999999999.0
        # calculate feature 7 - distance between curves
        f7 = really_big_number
        ci = curve1.points
        cj = curve2.points
        num_p = len(ci)
        for l in range(num_p):
            distance = 0
            for k in range(num_p):
                norm = np.linalg.norm(ci[k] - cj[(k + l) % num_p])
                distance = distance + norm * norm

            if distance < f7:
                f7 = distance
        f7 = np.sqrt(f7 / num_p)

        # calculate feature 8 - discrete curvature distance
        f8 = really_big_number
        ki = curve1._curvature
        kj = curve2._curvature
        num_p = len(ci)
        for l in range(num_p):
            distance = 0
            for k in range(num_p):
                norm = np.linalg.norm(ki[k] - kj[(k + l) % num_p])
                distance = distance + norm * norm

            if distance < f8:
                f8 = distance
        f8 = np.sqrt(f8)

        features_difference = np.concatenate((curve1.features - curve2.features, np.array([f7, f8])))
        return np.sqrt(np.dot(features_difference * Curve.A, features_difference))

    def to_json(self):
        c = {}
        c['points'] = self.points.tolist()
        c['features'] = self.features.tolist()
        return json.dumps(c)


#
# def random_noise():
#     return 1 + random.uniform(-1, 1) / 15

#
# # define pts from the question
# pts = [[6.55525, 3.05472],
#        [6.17284, 2.802609],
#        [5.53946, 2.649209],
#        [4.93053, 2.444444],
#        [4.32544, 2.318749],
#        [3.90982, 2.2875],
#        [3.51294, 2.221875],
#        [3.09107, 2.29375],
#        [2.64013, 2.4375],
#        [2.275444, 2.653124],
#        [2.137945, 3.26562],
#        [2.15982, 3.84375],
#        [2.20982, 4.31562],
#        [2.334704, 4.87873],
#        [2.314264, 5.5047],
#        [2.311709, 5.9135],
#        [2.29638, 6.42961],
#        [2.619374, 6.75021],
#        [3.32448, 6.66353],
#        [3.31582, 5.68866],
#        [3.35159, 5.17255],
#        [3.48482, 4.73125],
#        [3.70669, 4.51875],
#        [4.23639, 4.58968],
#        [4.39592, 4.94615],
#        [4.33527, 5.33862],
#        [3.95968, 5.61967],
#        [3.56366, 5.73976],
#        [3.78818, 6.55292],
#        [4.27712, 6.8283],
#        [4.89532, 6.78615],
#        [5.35334, 6.72433],
#        [5.71583, 6.54449],
#        [6.13452, 6.46019],
#        [6.54478, 6.26068],
#        [6.7873, 5.74615],
#        [6.64086, 5.25269],
#        [6.45649, 4.86206],
#        [6.41586, 4.46519],
#        [5.44711, 4.26519],
#        [5.04087, 4.10581],
#        [4.70013, 3.67405],
#        [4.83482, 3.4375],
#        [5.34086, 3.43394],
#        [5.76392, 3.55156],
#        [6.37056, 3.8778],
#        [6.53116, 3.47228]]
#
# for j in range(50):
#     for i in range(len(pts)):
#         for j in range(len(pts[i])):
#             pts[i][j] *= random_noise()
#
#     curve = Curve(pts)
#     print(curve.features)
#     curve.plot_curve()
#     plt.show()
