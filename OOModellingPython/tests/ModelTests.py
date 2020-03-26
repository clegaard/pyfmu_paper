import cProfile
import unittest
from pstats import SortKey

import numpy as np
import matplotlib.pyplot as plt

from oomodelling.examples.ExampleModels import MassDamper, MassSpringDamper, MassSpringDamperFlat, MSDTimeDep, \
    MSDAutonomous, \
    DelayExampleScenario, TwoMSDComparison, DelayRewireInput
from oomodelling.Model import Model
from oomodelling.ModelSolver import ModelSolver


class ModelTests(unittest.TestCase):
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

        ModelSolver().simulate(m, 0.0, 10, 0.01)

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
        ModelSolver().simulate(m, 0.0, 10.0, 0.01)
        # plt.plot(m.signals['time'], m.signals['x'])
        # plt.show()

        self.assertGreaterEqual(0.1, m.signals['x'][-1])

        self.assertTrue('x' in m.signals)
        self.assertTrue('der_x' in m.signals)
        self.assertTrue('spring' in m.signals)

    def test_record_states_MSD(self):
        m = MassSpringDamperFlat()
        sample_space = np.linspace(0.0, 10.0, 100)
        ModelSolver().simulate(m, 0.0, 10.0, 0.1, sample_space)
        delayed_signals = [m.x(-(10.0-t)) for t in sample_space]
        # plt.plot(m.signals['time'], m.signals['x'])
        # plt.plot(sample_space, delayed_signals)
        # plt.show()

        for (d, s) in zip(delayed_signals, m.signals['x']):
            assert np.isclose(d, s, atol=0.2, rtol=1e-10), (d,s)




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

        ModelSolver().simulate(m, 0.0, 10.0, 0.01)

        signals = ['x','v', 'der_x', 'der_v']
        for s in signals:
            for a, b in zip(m.m1.signals[s], m.m2.md.signals[s]):
                self.assertAlmostEqual(a, b)

    def test_msd_timedep(self):
        m = MSDTimeDep()
        ModelSolver().simulate(m, 0.0, 10, 0.01)
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
        m._update(X, 0.0)
        self.assertAlmostEqual(X[1], 1.0)
        self.assertAlmostEqual(X[2], 1.0)

        X = m.state_vector()
        self.assertAlmostEqual(X[1], 1.0)
        self.assertAlmostEqual(X[2], 1.0)

        m.reset()
        m.v = 1.0

        solver = ModelSolver()
        solver.simulate(m, 0.0, 10.0, 0.01)

        # plt.plot(m.signals['time'], m.signals['x'])
        # plt.show()

    def test_delay(self):
        m = DelayExampleScenario()
        ModelSolver().simulate(m, 0.0, 4.3, 0.01)
        # plt.plot(m.signals['time'], m.u.signals['F'])
        # plt.plot(m.signals['time'], m.d.signals['d'])
        # plt.show()
        self.assertAlmostEqual(m.u.signals['F'][-1], 4.0)
        self.assertAlmostEqual(m.d.signals['d'][-1], 0.0)

    def test_delay_rewire_input(self):
        m = DelayRewireInput()
        ModelSolver().simulate(m, 0.0, 5, 0.01)
        # plt.plot(m.signals['time'], m.d.signals['u'])
        # plt.plot(m.signals['time'], m.d.signals['d'])
        # plt.show()
        idx = int(0.5 / 0.01)
        self.assertTrue(m.d.signals['u'][idx] > 0.1)
        self.assertAlmostEqual(m.d.signals['d'][idx], 0.0)

    def test_search(self):
        sample = [0, 1, 2, 3, 4, 5, 6]
        self.assertEqual(0, Model._find_sup(0.1, sample))
        self.assertEqual(0, Model._find_sup(-1, sample))
        self.assertEqual(6, Model._find_sup(6.2, sample))
        self.assertEqual(3, Model._find_sup(3.8, sample))

    def test_change_model_var_on_the_fly_forbidden(self):
        m1 = MSDTimeDep()
        ModelSolver().simulate(m1, 0.0, 10, 0.01)
        # plt.plot(m1.u.signals['time'], m1.u.signals['F'])
        # plt.plot(m1.u.signals['time'], m1.msd.signals['x'])
        # plt.show()

        m2 = MSDTimeDep()

        def forbidden():
            m2.u.F = lambda: 0.0 if m2.u.time() < 1.0 else 4.0

        # Rewire the model after construction -- This will not affect the values read by the msd, because msd has stored the lambda of u at the time of the connection!
        self.assertRaises(AssertionError, forbidden)

        # SciPySolver(StepRK45).simulate(m2, 0.0, 10, 0.01)
        # plt.plot(m2.u.signals['time'], m2.u.signals['F'])
        # plt.plot(m2.u.signals['time'], m2.msd.signals['x'])
        # plt.show()
        # for a, b in zip(m1.msd.signals['x'], m2.msd.signals['x']):
        # Nothing changed in the input of m2.msd, and this is why we should not rewire stuff on the fly.
        # self.assertAlmostEqual(a, b, places=1)

    def test_change_model_input_on_the_fly_forbidden(self):
        m1 = MassSpringDamperFlat()
        m1.F = lambda: 0.0 if m1.time() < 4.0 else 4.0
        ModelSolver().simulate(m1, 0.0, 10, 0.01)
        # plt.plot(m1.signals['time'], m1.signals['F'])
        # plt.plot(m1.signals['time'], m1.signals['x'])
        # plt.show()

        m2 = MassSpringDamperFlat()
        m2.F = lambda: 0.0 if m2.time() < 1.0 else 4.0

        ModelSolver().simulate(m2, 0.0, 10, 0.01)
        # plt.plot(m2.signals['time'], m2.signals['F'])
        # plt.plot(m2.signals['time'], m2.signals['x'])
        # plt.show()

    def test_profile(self):
        cProfile.runctx('ModelSolver().simulate(TwoMSDComparison(), 0.0, 20.0, 0.01)', globals(), locals(), sort=SortKey.TIME)