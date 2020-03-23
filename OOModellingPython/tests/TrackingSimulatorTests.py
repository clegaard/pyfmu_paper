import unittest
from random import random, seed
import matplotlib.pyplot as plt

from ExampleTrackingSimulators import MSDTrackingSimulator
from ModelSolver import ModelSolver


class TrackingSimulatorTests(unittest.TestCase):

    def test_msd_tracking(self):
        seed(1)
        m = MSDTrackingSimulator()
        m.tolerance = 1
        m.horizon = 5.0
        m.nsamples = 20
        m.time_step = 0.1
        m.conv_xatol = 0.1
        m.conv_fatol = 0.1

        ModelSolver().simulate(m, 0.0, 50, 0.1)

        # _, (p1, p2, p3, p4) = plt.subplots(1, 4)
        # p1.plot(m.signals['time'], m.to_track.signals['F'], label='F')
        # p1.legend()
        # p2.plot(m.to_track.signals['time'], m.to_track.signals['x'], label='x')
        # p2.plot(m.tracking.signals['time'], m.tracking.signals['x'], label='~x')
        # for calib in m.recalibration_history:
        #     p2.plot(calib.ts, calib.xs[m.X_idx, :], '--', label='recall_x')
        # p2.legend()
        # p3.plot(m.signals['time'], m.signals['error'], label='error')
        # p3.legend()
        # p4.plot(m.to_track.signals['time'], m.to_track.signals['d'], label='d')
        # p4.plot(m.tracking.signals['time'], m.tracking.signals['d'], label='~d')
        # p4.legend()
        # plt.show()
        # print(m.tracking.signals['d'][-1])

        self.assertEqual(len(m.recalibration_history), 3)
        self.assertAlmostEqual(m.tracking.signals['d'][-1], 19.108687500000034)