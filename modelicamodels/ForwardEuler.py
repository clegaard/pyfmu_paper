from Model import Model
from Solver import Solver
import numpy as np

class ForwardEuler(Solver):
    def __init__(self):
        super().__init__()

    """
    Computes the behavior of the model.
    Uses a forward euler solver because we need to progagate intermediate signals to the different components.
    Operators such as delay need this information. 
    Using an off the shelf solver for this makes it impossible to get the real intermediate states, 
    due to the many intermediate model evaluations.
    """
    def simulate(self, model: Model, stop_t, h):
        model.reset()
        F = model.derivatives()
        ts = np.arange(0.0, stop_t, h)
        x = model.state_vector()
        x_next = np.zeros_like(x)
        m = len(x)
        for n in range(len(ts)):
            model.step_commit(t, x)
            t = ts[n]
            h = h
            ders = F(t, x)
            # FW Euler Step
            for i in range(m):
                x_next[i] = x[i] + ders[i]*h

            x = x_next
            x_next = np.zeros_like(x)



