import cProfile
import unittest

import matplotlib.pyplot as plt

from ExampleModels import MassDamper, MassSpringDamper, MassSpringDamperFlat, MSDTimeDep, MSDAutonomous, \
    DelayExampleScenario, TwoMSDComparison
from Model import Model
from SciPySolver import SciPySolver
from StepRK45 import StepRK45


class TestModel(unittest.TestCase):
    def test_some_model(self):
        m = MassDamper()

        self.assertEqual(m.x(), 0.0)
        self.assertEqual(m.v(), 1.0)
        self.assertEqual(len(m._states), 2+1)

    def test_msd(self):
        m = MassSpringDamper()
        self.assertEqual(m.nstates(), 3+1+1)
        init = m.state_vector()
        self.assertEqual(len(init), 3+1+1)

        SciPySolver(StepRK45).simulate(m, 0.0, 10, 0.01)

        self.assertEqual(len(m.signals), 2) # time and der_time
        self.assertEqual(len(m.md.signals), 8)
        self.assertEqual(len(m.s.signals), 4)
        self.assertTrue('der_x' in m.md.signals)
        self.assertTrue('F' in m.md.signals)
        self.assertTrue('F' in m.s.signals)
        self.assertGreaterEqual(0.1, m.md.signals['x'][-1])
        # plt.plot(m.signals['time'], m.md.signals['x'])
        # plt.show()

    def test_autonomous_model(self):
        m = MassSpringDamperFlat()
        SciPySolver(StepRK45).simulate(m, 0.0, 10.0, 0.01)
        # plt.plot(m.signals['time'], m.signals['x'])
        # plt.show()

        self.assertGreaterEqual(0.1, m.signals['x'][-1])

        self.assertTrue('x' in m.signals)
        self.assertTrue('der_x' in m.signals)
        self.assertTrue('spring' in m.signals)

    def test_model_with_autonomous(self):
        m = MassSpringDamperFlat()
        m.v = 2.0
        self.assertEqual(m._state_derivatives['x'](), 2.0)

        m._update([1.0, 1.0, 3.0], 0.0)
        self.assertEqual(m.time(), 1.0)
        self.assertEqual(m.x(), 1.0)
        self.assertEqual(m.v(), 3.0)

    def test_similar_states(self):
        m = TwoMSDComparison()

        SciPySolver(StepRK45).simulate(m, 0.0, 10.0, 0.01)

        signals = ['x','v', 'der_x', 'der_v']
        for s in signals:
            for a, b in zip(m.m1.signals[s], m.m2.md.signals[s]):
                self.assertAlmostEqual(a, b)


    def test_msd_timedep(self):
        m = MSDTimeDep()
        SciPySolver(StepRK45).simulate(m, 0.0, 10, 0.01)
        # plt.plot(m.u.signals['time'], m.u.signals['F'])
        # plt.show()
        self.assertGreaterEqual(m.u.signals['F'][-1], 3)
        self.assertGreaterEqual(m.msd.signals['x'][-1], 3)

    def test_pop(self):
        a = [1, 2, 3, 4]
        self.assertEqual(1, a.pop(0))
        self.assertEqual(2, a.pop(0))
        self.assertEqual(3, a.pop(0))
        self.assertEqual(4, a.pop(0))

    def test_msd_state_signals(self):
        m = MSDAutonomous()

        X = m.state_vector()
        self.assertEqual(m.nstates(), 2+1)
        self.assertEqual(len(X), 2+1)
        self.assertAlmostEqual(X[0], 0.0)
        self.assertAlmostEqual(X[1], 0.0)
        self.assertAlmostEqual(X[2], 1.0)

        m.x = 1.0
        m.v = 0.0
        X = m.state_vector()
        self.assertAlmostEqual(X[1], 1.0)
        self.assertAlmostEqual(X[2], 0.0)

        m.reset()
        m.assert_initialized()
        X = m.state_vector()
        self.assertAlmostEqual(X[1], 0.0)
        self.assertAlmostEqual(X[2], 1.0)

        X[1] = 1.0
        m._update(X.tolist(), 0.0)
        self.assertAlmostEqual(X[1], 1.0)
        self.assertAlmostEqual(X[2], 1.0)

        X = m.state_vector()
        self.assertAlmostEqual(X[1], 1.0)
        self.assertAlmostEqual(X[2], 1.0)

        m.reset()
        m.v = 1.0

        solver = SciPySolver(StepRK45)
        solver.simulate(m, 0.0, 10.0, 0.01)

        # plt.plot(m.signals['time'], m.signals['x'])
        # plt.show()

    def test_delay(self):
        m = DelayExampleScenario()
        SciPySolver(StepRK45).simulate(m, 0.0, 4.3, 0.01)
        # plt.plot(m.signals['time'], m.u.signals['F'])
        # plt.plot(m.signals['time'], m.d.signals['d'])
        # plt.show()
        self.assertAlmostEqual(m.u.signals['F'][-1], 4.0)
        self.assertAlmostEqual(m.d.signals['d'][-1], 0.0)

    def test_search(self):
        sample = [0, 1, 2, 3, 4, 5, 6]
        self.assertEqual(0, Model._find_sup(0.1, sample))
        self.assertEqual(0, Model._find_sup(-1, sample))
        self.assertEqual(6, Model._find_sup(6.2, sample))
        self.assertEqual(3, Model._find_sup(3.8, sample))

    def test_profile(self):
        cProfile.runctx('SciPySolver(StepRK45).simulate(TwoMSDComparison(), 0.0, 15.0, 0.01)', globals(), locals())