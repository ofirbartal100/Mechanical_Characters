from assembly import *
import scipy.optimize


class ContinousParameterOptimization:

    def __init__(self, nearest_assembly, target_curve):
        self.nearest_assembly = nearest_assembly
        self.target_curve = target_curve

        # make sure target_curve and assembly_curve has same number of points
        self._T = 360

    def _objective(self):
        def objective_function(p):
            assembly = Assembly(p)

            # aligns the estimated curve
            assembly.align_with_curve(self.target_curve)

            sum = 0
            for t in range(self._T):
                # get_curve_at calculates s_t as function of p at t

                sum += np.dot(
                    assembly.get_curve_at(t) - self.target_curve.get_curve_at(t),
                    assembly.get_curve_at(t) - self.target_curve.get_curve_at(t))

            return .5 * sum

        return objective_function

    def _jacobian(self):
        def gradient_function(p):
            assembly = Assembly(p)

            # aligns the estimated curve
            assembly.align_with_curve(self.target_curve)

            sum = 0
            for t in range(self._T):
                # get_curve_at calculates s_t as function of p at t
                doX_dop = assembly.get_red_point_dop()  # in R^(3xP)
                doX_doSt = assembly.get_red_point_doSt(t)
                doC_dop = assembly.get_const_dop()
                doC_doSt = assembly.get_const_doSt(t)
                doC_doSt_inv = np.linalg.inv(doC_doSt.T @ doC_doSt) @ doC_doSt.T
                doSt_dop = -doC_doSt_inv @ doC_dop

                #(doX_dop + doX_doSt @ doSt_dop) is in R^(3xP)
                # assembly.get_curve_at(t) - self.target_curve.get_curve_at(t)) is in R^3
                sum += (doX_dop + doX_doSt @ doSt_dop) @ (assembly.get_curve_at(t) - self.target_curve.get_curve_at(t))

            return sum

        return gradient_function

    def optimize_parameters(self):
        p = self.nearest_assembly.get_p()
        ret = scipy.optimize.minimize(self._objective(), p, jac=self._jacobian(), method='L-BFGS-B')
        if ret.success:
            return ret['x']
        else:
            raise Exception("did not converge")
