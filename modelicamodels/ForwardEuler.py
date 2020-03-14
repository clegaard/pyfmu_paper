from Solver import Solver
import numpy as np


class ForwardEuler(Solver):
    def __init__(self):
        super().__init__()

    def step(self, t, x, h, f):
        x_next = np.zeros_like(x)
        m = len(x)
        ders = f(t, x)
        # FW Euler Step
        for i in range(m):
            x_next[i] = x[i] + ders[i]*h
        return x_next
