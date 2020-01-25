import json
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as alg
import poly_point_isect
from shapely.geometry import Polygon
from sklearn.decomposition import PCA


class Curve:
    '''
    this class represents a curve by its points and its features
    '''
    # A = np.array([0.1212501165, 0.0005685005, 0, 1.640562057,0, 0.555359019205, 0.051260115736, 0], dtype=np.float64)
    A = np.array([1, 1, 1, 0, 0, 1, 1, 1], dtype=np.float64)

    def __init__(self, normalized_coordinates):
        self.features = np.zeros((6,), dtype=np.float64)

        if not isinstance(normalized_coordinates, np.ndarray):
            self.points = np.array(normalized_coordinates, dtype=np.float64)
        else:
            self.points = normalized_coordinates

        self._calculate_features()
        self._calculate_curvature()

    def plot_curve(self, custom_fig=None):
        if custom_fig is not None:
            fig, ax = custom_fig
        else:
            fig, ax = plt.subplots()
        curve = Polygon(self.points)
        x, y = curve.exterior.xy
        ax.plot(x, y)
        return (fig, ax)

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

    def plot(self, path=r"C:\Users\A\Desktop\temp", save_image=False):
        # evenly sampled time at 200ms intervals
        xs = [i[0] for i in self.points]
        ys = [i[1] for i in self.points]

        # red dashes, blue squares and green triangles
        plt.plot(xs, ys)
        if save_image:
            plt.savefig(path + fr"\curve.png")
        plt.show()
