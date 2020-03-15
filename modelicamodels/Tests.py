import unittest
import matplotlib.pyplot as plt

from BikeModel import BikeModel
from BikeModelWithDriver import BikeModelWithDriver
from BikeModelsWithDriver import BikeModelsWithDriver
from DynamicBikeModelWithDriver import DynamicBikeModelWithDriver
from DynamicDriver import DynamicDriver
from SciPySolver import SciPySolver
from StepRK45 import StepRK45


class TestExperiments(unittest.TestCase):
    def test_driver(self):
        m = DynamicDriver()

        SciPySolver(StepRK45).simulate(m, 10, 0.1)

        plt.plot(m.signals['time'], m.signals['steering'])
        plt.show()

    def test_bike_model_fixed_steering(self):
        m = BikeModel()
        m.deltaf = lambda: 0.4 if m.time() > 5.0 else 0.0
        SciPySolver(StepRK45).simulate(m, 10, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)
        p1.plot(m.signals['time'], m.signals['psi'])
        p2.plot(m.signals['x'], m.signals['y'])
        plt.show()

    def test_bike_model_with_driver(self):
        m = BikeModelWithDriver()

        SciPySolver(StepRK45).simulate(m, 200, 0.1)

        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.driver.signals['steering'])
        p2.plot(m.bike.signals['x'], m.bike.signals['y'])
        plt.show()

    def test_dynamic_bike_model_with_driver(self):
        m = DynamicBikeModelWithDriver()

        # ts = arange(0, 10.0, 0.01)
        # sol = odeint(m.derivatives(), m.state_vector(), ts, hmax=0.01, tfirst=True)
        # plt.plot(ts, sol[:, 5])
        # plt.show()

        m.reset()
        SciPySolver(StepRK45).simulate(m, 10.0, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.ddriver.signals['steering'])
        # p1.plot(m.signals['time'], m.dbike.signals['deltaf'])
        # p1.plot(m.signals['time'], m.dbike.signals['af'])
        # p1.plot(m.signals['time'], m.dbike.signals['y'])
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'])
        plt.show()

    def test_twobikes(self):
        m = BikeModelsWithDriver()
        SciPySolver(StepRK45).simulate(m, 10, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.ddriver.signals['steering'])
        p1.plot(m.signals['time'], m.kdriver.signals['steering'])
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'])
        p2.plot(m.kbike.signals['x'], m.kbike.signals['y'])
        plt.show()

    def calibrate_kinematics_driver(self):
        def cost(delay, k):
            m = BikeModelsWithDriver()
            SciPySolver(StepRK45).simulate(m, 10, 0.1)
            

        _, (p1, p2) = plt.subplots(1, 2)
        p1.plot(m.signals['time'], m.ddriver.signals['steering'])
        p1.plot(m.signals['time'], m.kdriver.signals['steering'])
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'])
        p2.plot(m.kbike.signals['x'], m.kbike.signals['y'])
        plt.show()