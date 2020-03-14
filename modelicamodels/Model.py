import math
from collections import namedtuple
from functools import reduce
from typing import Any, Iterator

from scipy.integrate import odeint, solve_ivp
import numpy as np


class Model:
    TIME = 'time'

    def __init__(self):
        super().__init__()
        self._under_construction = True
        self._states = []
        self._inputs = []
        self._parameters = []
        self._vars = []
        self._models = []
        self._initial_values = {}
        self._current_state_values = {}
        self._state_derivatives = {}
        self.signals = {}
        self.state(self.TIME, 0.0)
        self.der(self.TIME, lambda: 1.0)

    def _new_signal(self, name):
        assert name not in self.signals.keys()
        self.signals[name] = []

    def state(self, name, init):
        assert self._under_construction
        assert name.isidentifier()
        assert not callable(init)
        assert not hasattr(self, name)

        self._states.append(name)
        self._initial_values[name] = init
        self._current_state_values[name] = init

        self._new_signal(name)

        setattr(self, name, self._get_state_function(name))

    def der(self, state, fun):
        assert self._under_construction
        assert callable(fun)
        assert hasattr(self, state)
        der_name = self._der(state)
        self._new_signal(der_name)
        self._state_derivatives[state] = self._get_signal_function(der_name, fun)

    def input(self, name):
        assert self._under_construction
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._inputs.append(name)
        self._new_signal(name)
        setattr(self, name, self._get_signal_function(name, lambda: 0.0))

    def var(self, name, fun):
        assert self._under_construction
        assert callable(fun)
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._vars.append(name)
        self._new_signal(name)
        setattr(self, name, self._get_signal_function(name, fun))

    def parameter(self, name, val):
        assert name.isidentifier()
        assert not callable(val)
        assert not hasattr(self, name)
        self._parameters.append(name)
        setattr(self, name, val)

    def model(self, name, obj):
        assert self._under_construction
        assert not callable(obj)
        assert name.isidentifier()
        assert not hasattr(self, name)
        self._models.append(name)
        setattr(self, name, obj)

    def connect(self, in_m, in_p, out_m, out_p):
        assert self._under_construction

        def resolve(d=None):
            trg = getattr(out_m, out_p)
            # works for states, parameters, and also other inputs
            if callable(trg):
                return trg(d)
            else:
                return trg

        setattr(in_m, in_p, resolve)

    def save(self):
        assert self._under_construction
        self._under_construction = False

    """
    Sets every state to its initial value (provided at the constructor) and clears every signal to its initial value (or empty).
    If you want different initial values, then set them after calling this function.
    """
    def reset(self):
        assert not self._under_construction

        def reset_signal(s):
            signal = self.signals[s]
            signal.clear()
            if s in self._current_state_values.keys():
                self._current_state_values[s] = self._initial_values[s]

        self._proc_signals(reset_signal,
                           lambda m: getattr(self, m).reset())

    """
    Ensures that every signal has a single initial value, or is empty.
    """
    def assert_initialized(self):

        def _assert_initialized(s):
            assert s in self.signals.keys()
            assert len(self.signals[s]) == 0

        self._proc_signals(_assert_initialized,
                           lambda m: getattr(self, m).assert_initialized())

    def nstates(self):
        assert not self._under_construction
        nstates_models = sum([getattr(self, m).nstates() for m in self._models])
        return nstates_models + len(self._states)

    def nsignals(self):
        assert not self._under_construction
        totallist = [1,
                     len(self._states) * 2,
                     len(self._inputs),
                     len(self._vars)
                     ] + [getattr(self, m).nsignals() for m in self._models]

        return sum(totallist)

    def signal_names(self, prefix=''):
        assert not self._under_construction
        return self._fmap_signals(lambda s: prefix + s,
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
            self._update(npstate.tolist(), t)
            ders = self._compute_derivatives()
            return ders

        return model

    """
    Takes a flat state, and propagates it to the internal models.
    Destroys the given state!
    """
    def _update(self, state, t):
        assert not self._under_construction
        assert len(state) == self.nstates()

        popped, state = self._pop(state, len(self._states))

        for s, v in zip(self._states, popped):
            setattr(self, s, v)

        for m in self._models:
            model = getattr(self, m)
            popped, state = self._pop(state, model.nstates())
            model._update(popped, t)

        assert len(state) == 0

    """
    Stores a new snapshot in the state history.
    """
    def step_commit(self, state, t):
        self._update(state.tolist(), t)
        internal_time = self.time()
        assert np.isclose(internal_time, t)
        self._step_commit()

    # noinspection PyProtectedMember
    def _step_commit(self):
        current_length = self.get_history_size()

        def _commit_signal(name, value):
            self.signals[name].append(value)

        self.__proc_signals(lambda s: _commit_signal(s, self._current_state_values[s]),
                            lambda d: _commit_signal(self._der(d), self._state_derivatives[d]()),
                            lambda u: _commit_signal(u, getattr(self, u)()),
                            lambda v: _commit_signal(v, getattr(self, v)()),
                            lambda m: getattr(self, m)._step_commit())
        assert self.get_history_size() == current_length + 1

    def get_history_size(self):
        size = len(self.signals[self.TIME])

        def assertsize_signal(signal):
            assert len(self.signals[self.TIME]) == size

        def assertsize_model(model):
            assert getattr(self, model).get_history_size() == size

        self._proc_signals(assertsize_signal, assertsize_model)

        return size

    def current_signals(self):
        assert not self._under_construction
        return self._fmap_signals(lambda s: getattr(self, s)(),
                                  lambda m: getattr(self, m).current_signals())

    def _get_state_function(self, name):
        assert self._under_construction

        def signal(d=None):
            if d is None:
                return self._current_state_values[name]
            else:
                return self._delayed_signal_value(name, d)

        return signal

    def _get_signal_function(self, name, fun):
        assert self._under_construction

        def signal(d=None):
            if d is None:
                return fun()
            else:
                return self._delayed_signal_value(name, d)

        return signal

    def _delayed_signal_value(self, name, d):
        assert name in self.signals.keys()
        t = self.time()
        ts = max(0, t+d)  # if -d goes beyond, we set ts=0
        idx = self._earliest_time(ts)
        return self.signals[name][idx]

    """
    Searches for the index of timestamp that is closest (from the left) to the argument t.
    Examples:
        [0,1,2,3,4] , 1.2 -> 1
        [0,1,2,3,4] , 4.2 -> 4
    """
    def _earliest_time(self, t):
        ts = self.signals['time']
        return self._find_sup(t, ts)

    def _signals_from_state(self, npstate, t):
        self._update(npstate.tolist(), t)
        return self.current_signals()

    # noinspection PyProtectedMember
    def _compute_derivatives(self):
        # Assumes that state has been updated.
        res = self._fmap_states(lambda s: self._state_derivatives[s](),
                                lambda m: getattr(self, m)._compute_derivatives())
        return res

    @staticmethod
    def _der(s):
        return 'der_' + s

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
        total = [np.array(internal_data)] + models_data
        total_flat = reduce(lambda a, b: np.concatenate((a, b)), total)
        assert len(total_flat) == self.nstates()
        return total_flat

    def __proc_signals(self, pstate, pder, pin, pvar, pmodel):
        for s in self._states:
            pstate(s)
            pder(s)
        for u in self._inputs:
            pin(u)
        for v in self._vars:
            pvar(v)
        for m in self._models:
            pmodel(m)

    def _proc_signals(self, psignal, pmodel):
        self.__proc_signals(psignal,
                            lambda d: psignal(self._der(d)),
                            psignal,
                            psignal,
                            pmodel
                            )

    def _fmap_signals(self, f_signal, f_models):
        res = []
        for s in self._states:
            res.append(f_signal(s))
            res.append(f_signal(self._der(s)))
        for u in self._inputs:
            res.append(f_signal(u))
        for v in self._vars:
            res.append(f_signal(v))

        res_models = [f_models(m) for m in self._models]
        total = [np.array(res)] + res_models
        total_flat = reduce(lambda a, b: np.concatenate(a,b), total)

        num = self.nsignals()
        assert len(total_flat) == num
        return total_flat

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
            if key in self._current_state_values.keys():
                self._current_state_values[key] = value
            else:
                assert key in self._inputs or key in self._parameters
                super().__setattr__(key, value)

    @staticmethod
    def _find_sup(t, ts):
        idx = len(ts) - 1  # Start at the end
        while ts[idx] > t and idx > 0:
            idx -= 1
        assert (idx == len(ts) - 1 or idx == 0 or (ts[idx] <= t and ts[idx + 1] > t))
        assert 0 <= idx <= len(ts)-1
        return idx
