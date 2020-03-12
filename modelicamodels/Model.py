import math
from functools import reduce

from scipy.integrate import odeint
import numpy as np


class Model:
    def __init__(self):
        super().__init__()
        self._states = []
        self._inputs = []
        self._models = []

    def state(self, name, init):
        assert not hasattr(self, name)
        self._states.append(name)
        setattr(self, name, init)

    @staticmethod
    def _der(s):
        return 'der_'+s

    def der(self, state, fun):
        assert hasattr(self, state)
        assert not hasattr(self, self._der(state))
        setattr(self, self._der(state), fun)

    def input(self, name):
        assert not hasattr(self, name)
        self._inputs.append(name)
        setattr(self, name, lambda: 0.0)

    def var(self, name, fun):
        assert not hasattr(self, name)
        setattr(self, name, fun)

    def parameter(self, name, val):
        assert not hasattr(self, name)
        setattr(self, name, val)

    def model(self, name, obj):
        assert not hasattr(self, name)
        self._models.append(name)
        setattr(self, name, obj)

    @staticmethod
    def connect(in_m, in_p, out_m, out_p):
        def resolve():
            trg = getattr(out_m, out_p)
            # works for states, parameters, and also other inputs
            if callable(trg):
                return trg()
            else:
                return trg

        setattr(in_m, in_p, resolve)

    def nstates(self):
        nstates_models = sum([getattr(self, m).nstates() for m in self._models])
        return nstates_models + len(self._states)

    """
    Creates a flat state from the internal state and models
    """
    def initial(self):
        internal_initial = [getattr(self, s) for s in self._states]
        models_initials = [getattr(self, m).initial() for m in self._models]
        total_state = [internal_initial] + models_initials
        total_state_flat = reduce(lambda a, b: a+b, total_state)
        assert len(total_state_flat) == self.nstates()
        return total_state_flat

    def _pop(self, state, n):
        assert n <= len(state)
        popped = []
        for i in range(n):
            popped.append(state.pop(0))
        return popped, state

    """
    Takes a flat state, and propagates it to the internal models
    """
    def update(self, state):
        assert len(state) == self.nstates()

        popped, state = self._pop(state, len(self._states))

        for s, v in zip(self._states, popped):
            setattr(self, s, v)

        for m in self._models:
            model = getattr(self, m)
            popped, state = self._pop(state, model.nstates())
            model.update(popped)

        assert len(state) == 0

    def compute_derivatives(self):
        # Assumes that state has been updated.
        ders = [getattr(self, self._der(s))() for s in self._states]
        models_ders = [getattr(self, m).compute_derivatives() for m in self._models]

        total = [ders] + models_ders
        total_flat = reduce(lambda a, b: a + b, total)
        assert len(total_flat) == self.nstates()
        return total_flat

        assert len(ders) == self.nstates()
        return ders

    def derivatives(self):
        def model(npstate, t):
            state = npstate.tolist()
            # Map state to internal state
            self.update(state)
            return self.compute_derivatives()
        return model
