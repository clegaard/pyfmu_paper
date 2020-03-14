import math
from collections import namedtuple
from functools import reduce
from typing import Any, Iterator

from scipy.integrate import odeint, solve_ivp
import numpy as np


class Model:
    def __init__(self):
        super().__init__()
        self._under_construction = True
        self._states = []
        self._inputs = []
        self._parameters = []
        self._vars = []
        self._models = []

    def save(self):
        assert self._under_construction
        self._under_construction = False

    def state(self, name, init):
        assert self._under_construction
        assert name.isidentifier()
        assert not hasattr(self, name)
        assert not hasattr(self, self._hist(name))

        self._states.append(name)

        setattr(self, self._hist(name), [init])
        setattr(self, name, self._get_signal_function(name))

    def __setattr__(self, key, value):
        if key == '_under_construction' or self._under_construction:
            super().__setattr__(key, value)
        else:
            """
            Overrides the assignment operation to something sensible.
            - When a state is assigned, this is actually the current value of the state that is being assigned, 
                and not the state function.
            - When an input is assigned, this is the input function that is assigned, so this does not need special treatment.
            """
            assert hasattr(self, key)
            if key in self._states:
                state_hist = getattr(self, self._hist(key))
                state_hist[-1] = value
            else:
                assert key in self._inputs or key in self._parameters
                super().__setattr__(key, value)

    def _hist(self, s):
        return 'hist_' + s

    def _get_signal_function(self, name):
        assert self._under_construction
        def signal(d=None):
            if d is None:
                return getattr(self, self._hist(name))[-1]
            else:
                assert False, "TODO"
        return signal

    def der(self, state, fun):
        assert self._under_construction
        assert hasattr(self, state)
        assert not hasattr(self, self._der(state))
        setattr(self, self._der(state), fun)

    def input(self, name):
        assert False, "TODO"
        assert self._under_construction
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._inputs.append(name)
        setattr(self, name, lambda: 0.0)

    def var(self, name, fun):
        assert False, "TODO"
        assert self._under_construction
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._vars.append(name)
        setattr(self, name, fun)

    def parameter(self, name, val):
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._parameters.append(name)
        setattr(self, name, val)

    def model(self, name, obj):
        assert False, "TODO"
        assert self._under_construction
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._models.append(name)
        setattr(self, name, obj)

    def connect(self, in_m, in_p, out_m, out_p):
        assert self._under_construction
        def resolve():
            trg = getattr(out_m, out_p)
            # works for states, parameters, and also other inputs
            if callable(trg):
                return trg()
            else:
                return trg

        setattr(in_m, in_p, resolve)

    def nstates(self):
        assert not self._under_construction
        nstates_models = sum([getattr(self, m).nstates() for m in self._models])
        return nstates_models + len(self._states)

    def nsignals(self):
        assert not self._under_construction
        totallist = [1,
                    len(self._states)*2,
                    len(self._inputs),
                    len(self._vars)
                    ] + [getattr(self, m).nsignals() for m in self._models]

        return sum(totallist)

    def signal_names(self, prefix=''):
        assert not self._under_construction
        return self._fmap_signals(lambda s: prefix + s,
                                  lambda c: prefix + c,
                                  lambda m: getattr(self, m).signal_names(prefix + m + '.'))

    """
    Creates a flat state from the internal state and models
    """
    def state_vector(self):
        assert not self._under_construction
        return self._fmap_states(lambda s: getattr(self, s)(),
                                 lambda m: getattr(self, m).state_vector())

    def derivatives(self):
        assert not self._under_construction
        def model(t, npstate):
            assert not self._under_construction
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
        assert not self._under_construction
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
    Takes a flat state, and propagates it to the internal models
    """
    def update(self, state, t):
        assert not self._under_construction
        assert len(state) == self.nstates()

        popped, state = self._pop(state, len(self._states))

        for s, v in zip(self._states, popped):
            setattr(self, s, v)

        for m in self._models:
            model = getattr(self, m)
            popped, state = self._pop(state, model.nstates())
            model.update(popped, t)

        assert len(state) == 0

    def current_signals(self):
        assert not self._under_construction
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
        res = []
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
