import math
from collections import namedtuple
from functools import reduce
from typing import Any, Iterator

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

    def nsignals(self):
        totallist = [1,
                    len(self._states)*2,
                    len(self._inputs),
                    len(self._vars)
                    ] + [getattr(self, m).nsignals() for m in self._models]

        return sum(totallist)

    def signal_names(self, prefix=''):
        return self._fmap_signals(lambda s: prefix + s,
                                  lambda c: prefix + c,
                                  lambda m: getattr(self, m).signal_names(prefix + m + '.'))

    """
    Creates a flat state from the internal state and models
    """
    def initial(self):
        return self._fmap_states(lambda s: getattr(self, s),
                                 lambda m: getattr(self, m).initial())

    def derivatives(self):
        def model(npstate, t):
            # Map state to internal state
            self.update(npstate.tolist(), t)
            ders = self._compute_derivatives()
            return ders
        return model

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
            assert n not in res
            res[n] = []

        # Populate dict

        signals_raw = [self._signals_from_state(s, t) for s, t in zip(state_traj, ts)]
        for x in signals_raw:
            for n, v in zip(names, x):
                res[n].append(v)

        for n in names:
            assert len(res[n]) == len(state_traj)

        return res

    """
    Computes the behavior of the model
    """
    def simulate(self, stop_t, step_t):
        ts = np.arange(0.0, stop_t, step_t)
        # Note that bounding the step size of the solver is very important.
        # Otherwise, important input trajectories might be missed. Took me a few good hours to figure this out xD
        sol = odeint(self.derivatives(), self.initial(), ts, hmax=step_t)
        signals = self.signals(ts, sol)
        return signals

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

    def current_signals(self):
        return self._fmap_signals(lambda s: getattr(self, s),
                                  lambda c: getattr(self, c)(),
                                  lambda m: getattr(self, m).current_signals())

    def _signals_from_state(self, npstate, t):
        self.update(npstate.tolist(), t)
        return self.current_signals()

    # noinspection PyProtectedMember
    def _compute_derivatives(self):
        # Assumes that state has been updated.
        res = self._fmap_states(lambda s: getattr(self, self._der(s))(),
                                lambda m: getattr(self, m)._compute_derivatives())
        return res

    @staticmethod
    def _der(s):
        return 'der_'+s

    @staticmethod
    def _pop(state, n):
        assert n <= len(state)
        popped = []
        for i in range(n):
            popped.append(state.pop(0))
        return popped, state

    def _fmap_states(self, f_states, f_models):
        internal_data = [f_states(s) for s in self._states]
        models_data = [f_models(m) for m in self._models]
        total = [internal_data] + models_data
        total_flat = reduce(lambda a, b: a + b, total)
        assert len(total_flat) == self.nstates()
        return total_flat

    def _fmap_signals(self, f_states, f_callable, f_models):
        res = [f_states('time')]
        for s in self._states:
            res.append(f_states(s))
            res.append(f_callable(self._der(s)))
        for u in self._inputs:
            res.append(f_callable(u))
        for v in self._vars:
            res.append(f_callable(v))

        res_models = [f_models(m) for m in self._models]
        total = [res] + res_models
        total_flat = reduce(lambda a, b: a + b, total)

        num = self.nsignals()
        assert len(total_flat) == num
        return total_flat
