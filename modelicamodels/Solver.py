from Model import Model
import numpy as np


class Solver:
    def __init__(self):
        super().__init__()

    """
    Entry point for specific solvers.
    Does some housekeeping so that the solvers only have to worry about the stepping. 
    Using an off the shelf solver for this makes it impossible to get the real intermediate states, 
    due to the many intermediate model evaluations.
    """
    def simulate(self, model: Model, stop_t, h):
        assert np.isclose(model.time(), 0.0)
        model.assert_initialized()
        f = model.derivatives()
        ts = np.arange(0.0, stop_t, h)
        x = model.state_vector()
        for n in range(len(ts)):
            t = ts[n]
            model.step_commit(x, t)
            h = h
            x_next = self.step(t, x, h, f)
            x = x_next

    def step(self, t, x, h, f):
        assert False, "For subclasses"
        return np.zeros_like(x)
