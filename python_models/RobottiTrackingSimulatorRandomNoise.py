from oomodelling.Model import Model
from oomodelling.ModelSolver import ModelSolver
from oomodelling.TrackingSimulator import TrackingSimulator

from BikeDynamicModelSpeedDriven import BikeDynamicModelSpeedDriven
from DriverDynamic import DriverDynamic
from RobottiDriver import RobottiDriver
from RobottiDynamicModel import RobottiDynamicModel
import numpy as np


class RobottiTrackingSimulatorRandomNoise(TrackingSimulator):

    def __init__(self):
        super().__init__()

        self.driver = DriverDynamic()
        self.robot = RobottiDynamicModel()

        self.tracking = RobottiDynamicModel()

        self.robot.deltaFl = self.driver.steering
        self.robot.deltaFr = self.driver.steering
        self.tracking.deltaFl = self.driver.steering
        self.tracking.deltaFr = self.driver.steering

        self.match_signals(self.robot.X, self.tracking.X)
        self.match_signals(self.robot.Y, self.tracking.Y)

        self.X_idx = self.tracking.get_state_idx('X')
        self.Y_idx = self.tracking.get_state_idx('Y')

        self.save()

    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        new_caf = new_parameters[0]
        m = RobottiDynamicModel()
        # Set new parameter
        m.Caf = lambda: new_caf
        # Rewrite control input to mimic the past behavior.
        m.deltaFl = lambda: self.driver.steering(-(tf - m.time()))
        m.deltaFr = lambda: self.driver.steering(-(tf - m.time()))

        assert np.isclose(self.robot.X(-(tf - t0)), tracked_solutions[0][0])
        assert np.isclose(self.robot.Y(-(tf - t0)), tracked_solutions[1][0])
        # Initialize the state to the state at t0
        m.x = self.tracking.x(-(tf - t0))
        m.X = self.robot.X(-(tf - t0))
        m.y = self.tracking.y(-(tf - t0))
        m.Y = self.robot.Y(-(tf - t0))
        m.vy = self.tracking.vy(-(tf - t0))
        m.psi = self.tracking.psi(-(tf - t0))
        m.dpsi = self.tracking.dpsi(-(tf - t0))

        sol = ModelSolver().simulate(m, t0, tf, self.time_step, error_space)
        new_trajectories = sol.y
        if only_tracked_state:
            new_trajectories = np.array([
                sol.y[self.X_idx, :],
                sol.y[self.Y_idx, :]
            ])
            assert len(new_trajectories) == 2
            assert len(new_trajectories[0, :]) == len(sol.y[0, :])

        return new_trajectories

    def update_tracking_model(self, new_present_state, new_parameter):
        self.tracking.record_state(new_present_state, self.time(), override=True)
        self.tracking.Caf = lambda: new_parameter[0]
        assert np.isclose(new_present_state[self.X_idx], self.tracking.X())
        assert np.isclose(new_present_state[self.Y_idx], self.tracking.Y())

    def get_parameter_guess(self):
        return np.array([self.tracking.Caf()])


