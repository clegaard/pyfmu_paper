
import numpy as np

from BikeDynamicModel import BikeDynamicModel
from BikeKinematicModel import BikeKinematicModel
from DriverDynamic import DriverDynamic
from DriverKinematic import DriverKinematic
from Model import Model
from ModelSolver import ModelSolver
from TrackingSimulator import TrackingSimulator


class SystemToTrack(Model):

    def __init__(self):
        super().__init__()
        self.ddriver = DriverDynamic()
        self.dbike = BikeDynamicModel()

        self.dbike.deltaf = self.ddriver.steering

        self.x = self.var(self.dbike.X)
        self.y = self.var(self.dbike.Y)
        self.steering = self.var(self.ddriver.steering)

        self.save()


class TrackingModel(Model):
    def __init__(self):
        super().__init__()

        # Submodels
        self.kdriver = DriverKinematic()
        self.kbike = BikeKinematicModel()

        self.control_steering = self.input(lambda: 0.0)
        self.x = self.var(self.kbike.x)
        self.y = self.var(self.kbike.y)

        self.kdriver.u = self.control_steering
        self.kbike.deltaf = self.kdriver.steering

        self.save()


class BikeTrackingSimulatorKinematic(TrackingSimulator):
    def __init__(self):
        super().__init__()

        self.to_track = SystemToTrack()
        self.tracking = TrackingModel()

        self.tracking.control_steering = self.to_track.steering

        self.match_signals(self.to_track.x, self.tracking.x)
        self.match_signals(self.to_track.y, self.tracking.y)

        self.save()

    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        delay, k = new_parameters
        m = TrackingModel()
        m.kdriver.delay = delay
        m.kdriver.k = k
        # Rewrite control input to mimic the past behavior.
        m.control_steering = lambda d: self.to_track.steering(-(tf - m.time()))
        assert np.isclose(self.to_track.dbike.X(-(tf - t0)), tracked_solutions[0][0])
        assert np.isclose(self.to_track.dbike.Y(-(tf - t0)), tracked_solutions[1][0])
        m.kbike.x = self.to_track.dbike.X(-(tf - t0))
        m.kbike.y = self.to_track.dbike.Y(-(tf - t0))
        m.kbike.v = self.to_track.dbike.vx(-(tf - t0))
        m.kbike.psi = self.to_track.dbike.psi(-(tf - t0))

        sol = ModelSolver().simulate(m, t0, tf, self.time_step, error_space)
        new_trajectories = sol.y
        if only_tracked_state:
            new_trajectories = np.array([
                sol.y[3, :],
                sol.y[4, :]
            ])
            assert len(new_trajectories) == 2
            assert len(new_trajectories[0, :]) == len(sol.y[0, :])

        return new_trajectories

    def update_tracking_model(self, new_present_state):
        self.tracking.step(new_present_state, self.time(), override=True)
        assert np.isclose(new_present_state[3], self.tracking.x())
        assert np.isclose(new_present_state[4], self.tracking.y())

    def get_parameter_guess(self):
        return np.array([self.tracking.kdriver.delay, self.tracking.kdriver.k])