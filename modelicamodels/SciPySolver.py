from scipy.integrate import solve_ivp, RK45

from Model import Model
import numpy as np


class SciPySolver:
    def __init__(self, solverclass):
        self._solverclass = solverclass
        super().__init__()

    def simulate(self, model: Model, stop_t, h):
        assert np.isclose(model.time(), 0.0)
        model.assert_initialized()
        f = model.derivatives()
        x = model.state_vector()
        sol = solve_ivp(f, (0.0, stop_t), x, method=self._solverclass, max_step=h, model=model)
        assert sol.success
