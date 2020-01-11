from assembly import *


class ContinousParameterOptimization:

    def __init__(self,nearest_assembly,target_curve):
        self.nearest_assembly = nearest_assembly
        self.target_curve = target_curve

        # make sure target_curve and assembly_curve has same number of points
        self._T = 360


    def _objective(self):
        def objective_function(p):
            assembly = Assembly(p)

            # aligns the estimated curve
            assembly.align_with_curve(self.target_curve)

            sum=0
            for t in range(self._T):
                # get_curve_at calculates s_t as function of p at t

                sum += np.dot(
                    assembly.get_curve_at(t) - self.target_curve.get_curve_at(t),
                    assembly.get_curve_at(t) - self.target_curve.get_curve_at(t))

            return .5*sum

        return objective_function

    def _jacobian(self):
        def gradient_function(p):
            assembly = Assembly(p)

            # aligns the estimated curve
            assembly.align_with_curve(self.target_curve)

            sum=0
            for t in range(self._T):
                # get_curve_at calculates s_t as function of p at t

                sum += np.dot(
                    ,
                    assembly.get_curve_at(t) - self.target_curve.get_curve_at(t))

            return .5*sum

        return gradient_function