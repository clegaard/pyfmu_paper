from scipy.integrate import RK45

import numpy as np

from Model import Model


class StepRK45(RK45):

    def __init__(self, fun, t0, y0, t_bound, max_step=np.inf,
                 rtol=1e-3, atol=1e-6, vectorized=False,
                 first_step=None, **extraneous):
        assert max_step<np.inf
        self._model: Model = extraneous.pop('model')
        self._last_committed_t = t0
        super().__init__(fun, t0, y0, t_bound, max_step=max_step,
                 rtol=rtol, atol=atol, vectorized=vectorized,
                 first_step=first_step, **extraneous)

    def step(self):
        msg = super().step()
        assert msg is None
        step_taken = self.t - self._last_committed_t
        if step_taken >= self.max_step:  # Improves performance by not recording every sample.
            update_state = self._model.step(self.y, self.t)
            if update_state:
                self.y = self._model.state_vector()
            self._last_committed_t = self.t
