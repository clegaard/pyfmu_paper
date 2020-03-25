from oomodelling.Model import Model
from oomodelling.ModelSolver import ModelSolver
from oomodelling.TrackingSimulator import TrackingSimulator

from BikeDynamicModelSpeedDriven import BikeDynamicModelSpeedDriven
from RobottiDriver import RobottiDriver
from RobottiDynamicModel import RobottiDynamicModel
import numpy as np


class RobottiTrackingSimulator(TrackingSimulator):

    def __init__(self):
        super().__init__()

        self.driver = RobottiDriver()
        self.robot = RobottiDynamicModel()

        self.dbike = self.get_new_bike_model()

        self.dbike.vx = self.robot.vx
        self.dbike.deltaf = self.driver.steering
        self.robot.deltaFl = self.driver.steering
        self.robot.deltaFr = self.driver.steering

        self.match_signals(self.robot.X, self.dbike.X)
        self.match_signals(self.robot.Y, self.dbike.Y)

        self.X_idx = self.robot.get_state_idx('X')
        self.Y_idx = self.robot.get_state_idx('Y')

        self.save()

    def get_new_bike_model(self):
        b = BikeDynamicModelSpeedDriven()
        b.m = self.robot.m
        b.lf = self.robot.lf
        b.lr = self.robot.lr
        b.Iz = self.robot.Iz
        b.Caf = lambda: self.robot.Car
        b.Car = self.robot.Car
        return b

    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        new_caf = new_parameters[0]
        m = self.get_new_bike_model()
        # Set new parameter
        m.Caf = lambda: new_caf
        # Rewrite control input to mimic the past behavior.
        m.deltaf = lambda: self.driver.steering(-(tf - m.time()))
        m.vx = lambda: self.robot.vx(-(tf - m.time()))

        assert np.isclose(self.robot.X(-(tf - t0)), tracked_solutions[0][0])
        assert np.isclose(self.robot.Y(-(tf - t0)), tracked_solutions[1][0])
        # Initialize the state to the state at t0
        # set the state that we can measure from the robot.
        m.x = self.dbike.x(-(tf - t0))
        m.X = self.robot.X(-(tf - t0))
        m.y = self.dbike.y(-(tf - t0))
        m.Y = self.robot.Y(-(tf - t0))
        m.vx = self.dbike.vx(-(tf - t0))
        m.vy = self.dbike.vy(-(tf - t0))
        m.psi = self.dbike.psi(-(tf - t0))
        m.dpsi = self.dbike.dpsi(-(tf - t0))

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
        self.dbike.record_state(new_present_state, self.time(), override=True)
        self.dbike.Caf = lambda: new_parameter[0]
        assert np.isclose(new_present_state[self.X_idx], self.dbike.X())
        assert np.isclose(new_present_state[self.Y_idx], self.dbike.Y())

    def get_parameter_guess(self):
        return np.array([self.dbike.Caf()])


