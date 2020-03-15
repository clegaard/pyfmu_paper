import unittest
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares

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

    def plot_twobikes(self, delay, k):
        m = BikeModelsWithDriver()
        m.kdriver.delay = delay
        m.kdriver.k = k
        SciPySolver(StepRK45).simulate(m, 33, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.ddriver.signals['steering'])
        p1.plot(m.signals['time'], m.kdriver.signals['steering'])
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'])
        p2.plot(m.kbike.signals['x'], m.kbike.signals['y'])
        plt.show()

    def plot_cost_two_bikes_range(self, delays, ks):
        for d in delays:
            ks_costs = []
            for k in ks:
                print(d, k)
                ks_costs.append(self.cost_two_bikes([d, k]))
            plt.plot(ks, ks_costs, label=str(d))
        plt.legend()
        plt.show()

    def test_plot_cost_two_bikes_range(self):
        self.plot_cost_two_bikes_range([0.1, 0.2, 0.3, 0.4], [0.7, 0.8, 0.9, 1.0])

    def test_twobikes(self):
        self.plot_twobikes(0.3, 0.7045283)

    def cost_two_bikes(self, p):
        delay, k = p
        m = BikeModelsWithDriver()
        m.kdriver.delay = delay
        m.kdriver.k = k
        SciPySolver(StepRK45).simulate(m, 33, 0.1)
        dX = np.array(m.dbike.signals['X'])
        dY = np.array(m.dbike.signals['Y'])
        kX = np.array(m.kbike.signals['x'])
        kY = np.array(m.kbike.signals['y'])
        errx = (dX - kX) ** 2
        erry = (dY - kY) ** 2
        return errx.sum() + erry.sum()

    def test_calibrate_kinematics_driver(self):
        def cost(p):
            delay, k = p
            m = BikeModelsWithDriver()
            m.kdriver.delay = delay
            m.kdriver.k = k
            SciPySolver(StepRK45).simulate(m, 33, 0.1)
            dX = np.array(m.dbike.signals['X'])
            dY = np.array(m.dbike.signals['Y'])
            kX = np.array(m.kbike.signals['x'])
            kY = np.array(m.kbike.signals['y'])
            errx = (dX - kX)
            erry = (dY - kY)
            ssd = np.concatenate((errx, erry))
            return ssd

        sol = least_squares(cost, [0.2, 0.7045283], bounds=([0.0, 0.0], [1.0, 1.0]), verbose=2)
        print(sol)
        assert sol.success

        delay, k = sol.x
        self.plot_twobikes(delay, k)
        # Best solution: array([0.3      , 0.7045283])
