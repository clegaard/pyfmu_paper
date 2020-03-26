from collections import namedtuple

import numpy as np
from scipy.optimize import minimize

from oomodelling.Model import Model

CalibrationInfo = namedtuple('CalibrationInfo', 'p ts xs')


class TrackingSimulator(Model):
    def __init__(self):
        super().__init__()
        self.tolerance = self.parameter(10)
        self.conv_xatol = self.parameter(0.01)
        self.conv_fatol = self.parameter(1.0)
        self.horizon = self.parameter(1.0)
        self.max_iterations = self.parameter(100)
        self.cooldown = self.parameter(1.0)
        self.nsamples = self.parameter(10)
        self.time_step = self.parameter(0.1)
        self.error = self.var(self.get_error)
        self.last_calibration_time = self.time()
        self.recalibration_history = []
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
        error_space, tracked_solutions, actual_signals  = self.get_solution_over_horizon()
        error = self.compute_error(tracked_solutions, actual_signals)
        return error

    def compute_error(self, tracked_solutions, new_state_trajectories):
        assert len(tracked_solutions) == len(new_state_trajectories)
        sum = 0
        for (sol, actual) in zip(tracked_solutions, new_state_trajectories):
            sum += ((sol-actual)**2).sum()
        return sum

    def get_solution_over_horizon(self):
        t = self.time()
        max_horizon = min(t, self.horizon)  # We can't go beyong time 0
        error_space = np.linspace(t - max_horizon, t, self.nsamples)
        tracked_solutions = []
        actual_signals = []
        for (sol_signal, actual) in self._matched_signals:
            tracked_solutions.append(np.array([sol_signal(-(t - ti)) for ti in error_space]))
            actual_signals.append(np.array([actual(-(t - ti)) for ti in error_space]))
        assert len(tracked_solutions) == len(self._matched_signals) == len(actual_signals)
        return error_space, tracked_solutions, actual_signals

    def set_time(self, t):
        super().set_time(t)
        self.last_calibration_time = self.time()

    # noinspection PyUnreachableCode
    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        """
        Runs a new simulation of the tracking model starting from t0 till tf.
        Note that subclasses should not forget to set the appropriate inputs to the tracking model.
        :param only_tracked_state:
        :param new_parameters:
        :param t0:
        :param tf:
        :param tracked_solutions:
        :return: Returns the trajectories that are matched to the tracked model if only_tracked_state.
        Otherwise, returns the full state.
        """
        assert False, "For subclasses"
        return []

    # noinspection PyUnreachableCode
    def get_parameter_guess(self):
        assert False, "For subclasses"
        return np.zeros(2)

    def update_tracking_model(self, new_present_state, new_parameter):
        """
        Updates the model with the new state and best parameters found.
        :param new_present_state:
        :param new_parameter:
        :return:
        """
        assert False, "For subclasses"

    def recalibrate(self):
        """
        Starts a recalibration of the TrackingModel over the horizon.
        - Gets the tracked system trajectories over the horizon.
        - Starts a new optimization process to find the new parameters.
        - Changes the TrackingModel with the new parameters, so that the simulation may continue.
        :return:
        """
        error_space, tracked_solutions, _ = self.get_solution_over_horizon()
        t0 = error_space[0]
        tf = error_space[-1]

        def cost(p):
            new_trajs = self.run_whatif_simulation(p, t0, tf, tracked_solutions, error_space)
            error = self.compute_error(tracked_solutions, new_trajs)
            return error

        new_sol = minimize(cost, self.get_parameter_guess(), method='Nelder-Mead',
                 options={'xatol': self.conv_xatol, 'fatol': self.conv_fatol, 'maxiter': self.max_iterations})

        assert new_sol.success, new_sol.message
        # assert new_sol.fun < self.tolerance # Causes too many assertion errors.

        new_state_trajectories = self.run_whatif_simulation(new_sol.x, t0, tf, tracked_solutions, error_space, False)
        self.recalibration_history.append(CalibrationInfo(new_sol.x, error_space, new_state_trajectories))
        new_present_state = new_state_trajectories[:, -1]
        self.update_tracking_model(new_present_state, new_sol.x)

        self.last_calibration_time = self.time()
        return True

    def should_recalibrate(self):
        return (self.time() - self.last_calibration_time > self.cooldown) and self.error() > self.tolerance

    def discrete_step(self):
        """
        Overrides the Model.discrete_step in order to implement discrete time functionality.
        The general algorithm is:
        1. Check if the error has exceeded the recalibration threshold.
        2. If so, start recalibration.
        """
        internal_models_changed = super().discrete_step()
        if self.should_recalibrate():
            return self.recalibrate()
        return internal_models_changed


