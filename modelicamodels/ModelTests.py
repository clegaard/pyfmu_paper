import unittest

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

from Model import Model


class MassDamper(Model):
    def __init__(self):
        super().__init__()
        self.state('x', 0.0)
        self.state('vx', 1.0)

        self.parameter('d', 1.0)
        self.var('friction', lambda: self.d * self.vx)
        self.input('F')
        self.der('x', lambda: self.vx)
        self.der('vx', lambda: self.F() - self.friction())


class Spring(Model):
    def __init__(self):
        super().__init__()

        self.input('x')
        self.parameter('k', 1.0)
        self.var('F', lambda: - self.k * self.x())


class MassSpringDamper(Model):
    def __init__(self):
        super().__init__()

        self.model('md', MassDamper())
        self.model('s', Spring())

        self.connect(self.s, 'x', self.md, 'x')
        self.connect(self.md, 'F', self.s, 'F')


class MassSpringDamperFlat(Model):
    def __init__(self):
        super().__init__()
        self.state('x', 0.0)
        self.state('v', 1.0)

        self.parameter('k', 1.0)
        self.parameter('d', 1.0)
        self.var('spring', lambda: self.k * self.x)
        self.var('damper', lambda: self.d * self.v)
        self.der('x', lambda: self.v)
        self.der('v', lambda: - self.damper() - self.spring())


class TestModel(unittest.TestCase):
    def test_some_model(self):
        m = MassDamper()

        self.assertEqual(m.x, 0.0)
        self.assertEqual(m.vx, 1.0)
        self.assertEqual(len(m._states), 2)

    def test_msd(self):
        m = MassSpringDamper()
        init = m.initial()
        self.assertEqual(m.nstates(), 2)
        self.assertEqual(len(init), 2)

        ts = np.arange(0, 10, 0.01)
        sol = odeint(m.derivatives(), m.initial(), ts)
        # plt.plot(ts, sol)
        # plt.show()

        results = m.signals(ts, sol)
        self.assertEqual(len(results), 7+3+1)
        self.assertTrue('md.der_x' in results)
        self.assertTrue('md.F' in results)
        self.assertTrue('s.F' in results)

    def test_autonomous_model(self):
        m = MassSpringDamperFlat()
        ts = np.arange(0, 10, 0.01)
        sol = odeint(m.derivatives(), m.initial(), ts)
        # plt.plot(ts, sol)
        # plt.show()

        results = m.signals(ts, sol)
        self.assertEqual(len(results), 7)
        self.assertTrue('x' in results)
        self.assertTrue('der_x' in results)
        self.assertTrue('spring' in results)

        # plt.plot(results['time'], results['x'])
        # plt.plot(results['x'], results['v'])
        # plt.show()

    def test_model_with_autonomous(self):
        m = MassSpringDamperFlat()
        m.v = 2.0
        self.assertEqual(m.der_x(), 2.0)

        m.update([1.0, 3.0], 0.0)
        self.assertEqual(m.x, 1.0)
        self.assertEqual(m.v, 3.0)

    def test_similar_states(self):
        m1 = MassSpringDamperFlat()
        m2 = MassSpringDamper()
        ts = np.arange(0, 10, 0.01)
        sol1 = odeint(m1.derivatives(), m1.initial(), ts)
        sol2 = odeint(m2.derivatives(), m2.initial(), ts)
        # plt.plot(ts, sol1)
        # plt.plot(ts, sol2)
        # plt.show()

        for x1, x2 in zip(sol1, sol2):
            self.assertEqual(len(x1), len(x2))
            for i in range(len(x1)):
                self.assertTrue(np.isclose(x1[i], x2[i]))

    def test_pop(self):
        a = [1, 2, 3, 4]
        self.assertEqual(1, a.pop(0))
        self.assertEqual(2, a.pop(0))
        self.assertEqual(3, a.pop(0))
        self.assertEqual(4, a.pop(0))
