from scipy.integrate import RK45

import numpy as np


class StepRK45(RK45):

    def __init__(self, fun, t0, y0, t_bound, max_step=np.inf,
                 rtol=1e-3, atol=1e-6, vectorized=False,
                 first_step=None, **extraneous):
        self._model = extraneous.pop('model')
        super().__init__(fun, t0, y0, t_bound, max_step=max_step,
                 rtol=rtol, atol=atol, vectorized=vectorized,
                 first_step=first_step, **extraneous)

    def step(self):
        msg = super().step()
        assert msg is None
        self._model.step_commit(self.y, self.t)
