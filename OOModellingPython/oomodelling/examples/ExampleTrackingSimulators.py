import math
from random import random

import numpy as np

from oomodelling.Model import Model
from oomodelling.ModelSolver import ModelSolver
from oomodelling.TrackingSimulator import TrackingSimulator


class MSDFlatInDamper(Model):
    def __init__(self):
        super().__init__()
        self.x = self.state(0.0)
        self.v = self.state(0.0)

        self.F = self.input(lambda: math.sin(self.time()))

        self._rand_d = 1.0

        self.k = self.parameter(1.0)
        self.d = self.input(lambda: self._rand_d)
        self.spring = self.var(lambda: self.k * self.x())
        self.damper = self.var(lambda: self.d() * self.v())
        self.der('x', lambda: self.v())
        self.der('v', lambda: self.F() - self.damper() - self.spring())
        self.save()

    def random_d(self):
        return random()*self.time()

    def discrete_step(self):
        super().discrete_step()
        self._rand_d = self.random_d()
        return True


class MSDTrackingSimulator(TrackingSimulator):
    def __init__(self):
        super().__init__()

        self.to_track = MSDFlatInDamper()

        self.tracking = MSDFlatInDamper()
        self.tracking.d = lambda: 1.0

        self.match_signals(self.to_track.x, self.tracking.x)
        self.match_signals(self.to_track.v, self.tracking.v)

        self.X_idx = self.tracking.get_state_idx('x')
        self.V_idx = self.tracking.get_state_idx('v')

        self.save()

    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        new_d = new_parameters[0]
        m = MSDFlatInDamper()
        m.d = lambda: new_d
        # Rewrite control input to mimic the past behavior.
        m.F = lambda: self.to_track.F(-(tf - m.time()))
        assert np.isclose(self.to_track.x(-(tf - t0)), tracked_solutions[0][0])
        assert np.isclose(self.to_track.v(-(tf - t0)), tracked_solutions[1][0])
        m.x = self.to_track.x(-(tf - t0))
        m.v = self.to_track.v(-(tf - t0))

        sol = ModelSolver().simulate(m, t0, tf, self.time_step, error_space)
        new_trajectories = sol.y
        if only_tracked_state:
            new_trajectories = np.array([
                sol.y[self.X_idx, :],
                sol.y[self.V_idx, :]
            ])
            assert len(new_trajectories) == 2
            assert len(new_trajectories[0, :]) == len(sol.y[0, :])

        return new_trajectories

    def update_tracking_model(self, new_present_state, new_parameter):
        self.tracking.record_state(new_present_state, self.time(), override=True)
        self.tracking.d = lambda: new_parameter[0]
        assert np.isclose(new_present_state[self.X_idx], self.tracking.x())
        assert np.isclose(new_present_state[self.V_idx], self.tracking.v())

    def get_parameter_guess(self):
        return np.array([self.tracking.d()])