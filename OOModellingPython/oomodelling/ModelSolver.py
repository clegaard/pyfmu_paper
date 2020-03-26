import numpy as np
from scipy.integrate import solve_ivp, RK45

from oomodelling.Model import Model


class ModelSolver:
    def __init__(self):
        super().__init__()

    def simulate(self, model: Model, start_t, stop_t, h, t_eval=None):
        model.set_time(start_t)
        model.assert_initialized()
        f = model.derivatives()
        x = model.state_vector()
        # Record first time.
        model.record_state(x, start_t)
        sol = solve_ivp(f, (start_t, stop_t), x, method=StepRK45, max_step=h, model=model, t_eval=t_eval)
        assert sol.success
        return sol


class StepRK45(RK45):

    def __init__(self, fun, t0, y0, t_bound, max_step=np.inf,
                 rtol=1e-3, atol=1e-6, vectorized=False,
                 first_step=None, **extraneous):
        assert max_step<np.inf, "Max step is infinity."
        self._model: Model = extraneous.pop('model')
        super().__init__(fun, t0, y0, t_bound, max_step=max_step,
                 rtol=rtol, atol=atol, vectorized=vectorized,
                 first_step=first_step, **extraneous)

    def step(self):
        msg = super().step()
        assert msg is None, msg
        step_taken = self.t - self._model.get_last_commit_time()
        if step_taken >= self.max_step:  # Improves performance by not recording every sample.
            self._model.record_state(self.y, self.t)
            update_state = self._model.discrete_step()
            if update_state:
                self.y = self._model.state_vector()
            assert np.isclose(self.t, self._model.get_last_commit_time()), "Commit was made to the model."
