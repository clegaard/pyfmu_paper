from scipy.integrate import solve_ivp

from Model import Model


class SciPySolver:
    def __init__(self, solverclass):
        self._solverclass = solverclass
        super().__init__()

    def simulate(self, model: Model, start_t, stop_t, h, t_eval=None):
        model.set_time(start_t)
        model.assert_initialized()
        f = model.derivatives()
        x = model.state_vector()
        # Record first time.
        model.step(x, start_t)
        sol = solve_ivp(f, (start_t, stop_t), x, method=self._solverclass, max_step=h, model=model, t_eval=t_eval)
        assert sol.success
        return sol