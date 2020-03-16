import numpy as np
from scipy.optimize import minimize

from BikeDynamicModel import BikeDynamicModel
from BikeModel import BikeModel
from DynamicDriver import DynamicDriver
from KinematicsDriver import KinematicsDriver
from Model import Model
from SciPySolver import SciPySolver
from StepRK45 import StepRK45


class SystemToTrack(Model):

    def __init__(self):
        super().__init__()
        self.ddriver = DynamicDriver()
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
        self.kdriver = KinematicsDriver()
        self.kbike = BikeModel()

        self.control_steering = self.input()
        self.x = self.var(self.kbike.x)
        self.y = self.var(self.kbike.y)

        self.kdriver.u = self.control_steering
        self.kbike.deltaf = self.kdriver.steering

        self.save()



class TrackingSimulator(Model):
    def __init__(self):
        super().__init__()

        self.tolerance = self.parameter(10)
        self.horizon = self.parameter(1.0)
        self.nsamples = self.parameter(10)
        self.error = self.var(self.get_error)

        self.to_track = SystemToTrack()
        self.tracking = TrackingModel()

        self.tracking.control_steering = self.to_track.steering

        self.save()

    def get_solution_over_horizon(self):
        t = self.time()
        max_horizon = min(t, self.horizon)  # We can't go beyong time 0
        error_space = np.linspace(t - max_horizon, t, self.nsamples)
        tracked_x_traj = np.array([self.to_track.x(-(t-ti)) for ti in error_space])
        tracked_y_traj = np.array([self.to_track.y(-(t-ti)) for ti in error_space])
        return error_space, tracked_x_traj, tracked_y_traj

    def get_error(self):
        """
        Computes the error over the horizon.
        Discretizes the tracked signals over the horizon using the sample size given.
        :return:
        """
        t = self.time()
        error_space, tracked_x, tracked_y  = self.get_solution_over_horizon()
        actual_x = np.array([self.tracking.x(-(t-ti)) for ti in error_space])
        actual_y = np.array([self.tracking.y(-(t-ti)) for ti in error_space])
        error = self.compute_error(tracked_x, tracked_y, actual_x, actual_y)
        return error

    def compute_error(self, tracked_x, tracked_y, actual_x, actual_y):
        return ((tracked_x-actual_x)**2).sum() + ((tracked_y-actual_y)**2).sum()

    def recalibrate(self):
        """
        Starts a recalibration of the TrackingModel over the horizon.
        - Gets the tracked system trajectories over the horizon.
        - Starts a new optimization process to find the new parameters.
        - Changes the TrackingModel with the new parameters, so that the simulation may continue.
        :return:
        """
        error_space, tracked_x, tracked_y = self.get_solution_over_horizon()
        t_startcal = error_space[0]
        t_endcal = error_space[-1]
        t_recal = (t_endcal - t_startcal)

        def map_t(t_cal):
            """
            Returns the time in the outer simulation that currents to the given time in the inner calibration.
            Is used to recover the control input from the out simulation over the horizon.
            :param t_cal:
            :return:
            """
            assert 0.0 <= t_cal <= t_recal
            sim_t = t_startcal + t_cal
            assert t_startcal <= sim_t <= t_endcal
            return sim_t

        def cost(p):
            delay, k = p
            m = TrackingModel()

            m.kdriver.delay = delay
            m.kdriver.k = k
            m.control_steering = lambda d: self.to_track.steering(-(t_endcal - map_t(m.time())))
            assert np.isclose(self.to_track.dbike.X(-(t_endcal - t_startcal)), tracked_x[0])
            assert np.isclose(self.to_track.dbike.Y(-(t_endcal - t_startcal)), tracked_y[0])
            m.kbike.x = self.to_track.dbike.X(-(t_endcal - t_startcal))
            m.kbike.y = self.to_track.dbike.Y(-(t_endcal - t_startcal))
            m.kbike.v = self.to_track.dbike.vx(-(t_endcal - t_startcal))
            m.kbike.psi = self.to_track.dbike.psi(-(t_endcal - t_startcal))

            SciPySolver(StepRK45).simulate(m, t_recal, 0.1)
            actual_x = m.signals['x']
            actual_y = m.signals['y']
            error = self.compute_error(tracked_x, tracked_y, actual_x, actual_y)
            return error

        new_sol = minimize(cost, [self.tracking.kdriver.delay, self.tracking.kdriver.k], method='Nelder-Mead',
                 options={'xatol': 0.01, 'fatol': 1.0})

        # TODO
        print(new_sol)



    def step_commit(self, state, t):
        """
        Overrides the Model.step_commit in order to implement discrete time functionality.
        The general algorithm is:
        1. Check if the error has exceeded the recalibration threshold.
        2. If so, start recalibration.
        """
        super().step_commit(state, t)

        if self.error() > self.tolerance:
            self.recalibrate()
