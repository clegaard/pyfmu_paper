import math
from collections import namedtuple
from functools import reduce

from scipy.integrate import odeint
import numpy as np

class Model:
    def __init__(self):
        super().__init__()
        self._states = []
        self._inputs = []
        self._vars = []
        self._models = []
        self.time = 0.0

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
        self._vars.append(name)
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

    def _pop(self, state, n):
        assert n <= len(state)
        popped = []
        for i in range(n):
            popped.append(state.pop(0))
        return popped, state

    """
    Takes a flat state, and propagates it to the internal models
    """
    def update(self, state, t):
        assert len(state) == self.nstates()

        self.time = t

        popped, state = self._pop(state, len(self._states))

        for s, v in zip(self._states, popped):
            setattr(self, s, v)

        for m in self._models:
            model = getattr(self, m)
            popped, state = self._pop(state, model.nstates())
            model.update(popped, t)

        assert len(state) == 0

    def signal_names(self, prefix=''):
        res = []
        res.append('time')
        for s in self._states:
            res.append(s)
            res.append(self._der(s))
        for u in self._inputs:
            res.append(u)
        for v in self._vars:
            res.append(v)

        res_models = [getattr(self, m).signal_names(m + '.') for m in self._models]

        total = [res]+res_models
        total_flat = reduce(lambda a, b: a + b, total)
        total_flat_prefix = [prefix + s for s in total_flat]
        return total_flat_prefix

    def current_signals(self):
        res = [self.time]
        for s in self._states:
            res.append(getattr(self, s))
            res.append(getattr(self, self._der(s))())
        for u in self._inputs:
            res.append(getattr(self, u)())
        for v in self._vars:
            res.append(getattr(self, v)())

        res_models = [getattr(self, m).current_signals() for m in self._models]
        total = [res] + res_models
        total_flat = reduce(lambda a, b: a + b, total)
        return total_flat

    def signals_from_state(self, npstate, t):
        self.update(npstate.tolist(), t)
        return self.current_signals()

    """
    Creates a dictionary of signals.
    Anything that changes over time is a signal
    """
    def signals(self, ts, state_traj):
        assert len(ts) == len(state_traj)
        names = self.signal_names()

        # Init dict
        res = {}
        for n in names:
            res[n] = []

        # Populate dict

        signals_raw = [self.signals_from_state(s, t) for s, t in zip(state_traj, ts)]
        for x in signals_raw:
            for n, v in zip(names, x):
                res[n].append(v)

        for n in names:
            assert len(res[n]) == len(state_traj)

        return res

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

    def compute_derivatives(self):
        # Assumes that state has been updated.
        ders = [getattr(self, self._der(s))() for s in self._states]
        models_ders = [getattr(self, m).compute_derivatives() for m in self._models]

        total = [ders] + models_ders
        total_flat = reduce(lambda a, b: a + b, total)
        assert len(total_flat) == self.nstates()
        return total_flat

    def derivatives(self):
        def model(npstate, t):
            # Map state to internal state
            self.update(npstate.tolist(), t)
            return self.compute_derivatives()
        return model
