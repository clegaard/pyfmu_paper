import unittest
import matplotlib.pyplot as plt
from numpy.ma import arange
from scipy.integrate import odeint

from BikeModel import BikeModel
from BikeModelWithDriver import BikeModelWithDriver
from Driver import Driver


class TestExperiments(unittest.TestCase):
    def test_driver(self):
        m = Driver()
        ts = arange(0, 10, 0.01)
        sol = odeint(m.derivatives(), m.state_vector(), ts)

        results = m.signals(ts, sol)

        plt.plot(results['time'], results['steering'])
        plt.show()

    def test_bike_model_fixed_steering(self):
        m = BikeModel()
        m.deltaf = lambda: 0.4 if m.time > 5.0 else 0.0
        ts = arange(0, 200, 0.01)
        sol = odeint(m.derivatives(), m.state_vector(), ts)
        results = m.signals(ts, sol)

        _, (p1, p2) = plt.subplots(1, 2)
        p1.plot(results['time'], results['psi'])
        p2.plot(results['x'], results['y'])
        plt.show()

    def test_bike_model_with_driver(self):
        m = BikeModelWithDriver()

        results = m.simulate(200, 0.01)

        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(results['time'], results['driver.steering'])
        p2.plot(results['bike.x'], results['bike.y'])
        plt.show()


