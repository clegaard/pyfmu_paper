import numpy as np

from Model import Model
from TrackingBikeModels import SystemToTrack, TrackingModel


class TrackingSimulator(Model):
    def __init__(self):
        super().__init__()
        self.tolerance = self.parameter(10)
        self.horizon = self.parameter(1.0)
        self.nsamples = self.parameter(10)
        self.time_step = self.parameter(0.1)
        self.error = self.var(self.get_error)
        self.last_calibration_time = self.time()
        self._matched_signals = []

    def match_signals(self, solution, approximation):
        self._matched_signals.append((solution, approximation))

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

    def get_solution_over_horizon(self):
        t = self.time()
        max_horizon = min(t, self.horizon)  # We can't go beyong time 0
        error_space = np.linspace(t - max_horizon, t, self.nsamples)
        tracked_x_traj = np.array([self.to_track.x(-(t-ti)) for ti in error_space])
        tracked_y_traj = np.array([self.to_track.y(-(t-ti)) for ti in error_space])
        return error_space, tracked_x_traj, tracked_y_traj

    def set_time(self, t):
        super().set_time(t)
        self.last_calibration_time = self.time()

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
        t = error_space[-1]

        def get_new_tracking_trajectories(delay, k):
            m = TrackingModel()

            m.kdriver.delay = delay
            m.kdriver.k = k
            m.control_steering = lambda d: self.to_track.steering(-(t - m.time()))
            assert np.isclose(self.to_track.dbike.X(-(t - t_startcal)), tracked_x[0])
            assert np.isclose(self.to_track.dbike.Y(-(t - t_startcal)), tracked_y[0])
            m.kbike.x = self.to_track.dbike.X(-(t - t_startcal))
            m.kbike.y = self.to_track.dbike.Y(-(t - t_startcal))
            m.kbike.v = self.to_track.dbike.vx(-(t - t_startcal))
            m.kbike.psi = self.to_track.dbike.psi(-(t - t_startcal))

            sol = SciPySolver(StepRK45).simulate(m, t_startcal, t, self.time_step, error_space)
            return sol.y

        def cost(p):
            delay, k = p

            new_state_trajectories = get_new_tracking_trajectories(delay, k)

            # TODO: we should not have to use numbers to refer to states. We should use stuff like 'x' and 'y'. If
            #  anything changes in the model, the code below breaks.
            actual_x = new_state_trajectories[3]
            actual_y = new_state_trajectories[4]
            assert (np.isclose(actual_x[0], tracked_x[0]))
            assert (np.isclose(actual_y[0], tracked_y[0]))
            error = self.compute_error(tracked_x, tracked_y, actual_x, actual_y)
            return error

        new_sol = minimize(cost, [self.tracking.kdriver.delay, self.tracking.kdriver.k], method='Nelder-Mead',
                 options={'xatol': 0.01, 'fatol': 1.0})

        assert new_sol.success, new_sol.message

        newdelay, newk = new_sol.x
        new_state_trajectories = get_new_tracking_trajectories(newdelay, newk)
        self.recalibration_history.append(CalibrationInfo(newdelay, newk, error_space, new_state_trajectories))
        new_present_state = new_state_trajectories[:, -1]
        self.tracking.step(new_present_state, self.time(), override=True)
        assert np.isclose(new_present_state[3], self.tracking.x())
        assert np.isclose(new_present_state[4], self.tracking.y())

        self.last_calibration_time = self.time()
        return True

    def should_recalibrate(self):
        return (self.time() - self.last_calibration_time > self.horizon) and self.error() > self.tolerance

    def step(self, state, t, override=False):
        """
        Overrides the Model.step_commit in order to implement discrete time functionality.
        The general algorithm is:
        1. Check if the error has exceeded the recalibration threshold.
        2. If so, start recalibration.
        """
        assert not override
        state_updated = super().step(state, t, override)
        assert not state_updated

        if self.should_recalibrate():
            return self.recalibrate()
        return False


class ExampleTrackingSimulator(TrackingSimulator):
    def __init__(self):
        super().__init__()

        self.to_track = SystemToTrack()
        self.tracking = TrackingModel()

        self.tracking.control_steering = self.to_track.steering

        self.match_signals(self.to_track.x, self.tracking.x)
        self.match_signals(self.to_track.y, self.tracking.y)

        self.save()