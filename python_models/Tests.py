import logging
import unittest
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize_scalar
from random import seed

from BikeKinematicModel import BikeKinematicModel
from BikeKinematicModelWithDriver import BikeKinematicModelWithDriver
from BikeModelsWithDriver import BikeModelsWithDriver
from BikeDynamicModelWithDriver import BikeDynamicModelWithDriver
from BikeTrackingWithDynamic import BikeTrackingSimulatorDynamic
from BikeTrackingWithDynamicWithoutStateRestore import BikeTrackingWithDynamicWithoutStateRestore
from DriverDynamic import DriverDynamic
from oomodelling.ModelSolver import ModelSolver
from BikeTrackingWithKinematic import TrackingSimulator, BikeTrackingSimulatorKinematic
from RobottiBikeModelsWithDriver import RobottiBikeModelsWithDriver
from RobottiDynamicModelWithDriver import RobottiDynamicModelWithDriver
from RobottiTrackingSimulator import RobottiTrackingSimulator
from RobottiTrackingSimulatorRandomNoise import RobottiTrackingSimulatorRandomNoise


class TestExperiments(unittest.TestCase):
    def test_driver(self):
        m = DriverDynamic()

        ModelSolver().simulate(m, 0.0, 10, 0.1)

        plt.plot(m.signals['time'], m.signals['steering'])
        plt.show()

    def test_bike_model_fixed_steering(self):
        m = BikeKinematicModel()
        m.deltaf = lambda: 0.4 if m.time() > 5.0 else 0.0
        ModelSolver().simulate(m, 0.0, 10, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)
        p1.plot(m.signals['time'], m.signals['psi'])
        p2.plot(m.signals['x'], m.signals['y'])
        plt.show()

    def test_bike_model_with_driver(self):
        m = BikeKinematicModelWithDriver()

        ModelSolver().simulate(m, 0.0, 200, 0.1)

        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.driver.signals['steering'])
        p2.plot(m.bike.signals['x'], m.bike.signals['y'])
        plt.show()

    def test_dynamic_bike_model_with_driver(self):
        m = BikeDynamicModelWithDriver()
        m.dbike.Caf = lambda: 800
        m.reset()
        ModelSolver().simulate(m, 0.0, 40.0, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.ddriver.signals['steering'])
        # p1.plot(m.signals['time'], m.dbike.signals['af'])
        # p1.plot(m.signals['time'], m.dbike.signals['Fcf'])
        p2.scatter(m.dbike.signals['X'], m.dbike.signals['Y'], c=m.signals['time'])
        plt.show()

    def plot_twobikes(self, delay, k, nperiods, stoptime):
        m = BikeModelsWithDriver()
        m.ddriver.starttime = 10.0
        m.ddriver.nperiods = nperiods
        m.kdriver.delay = delay
        m.kdriver.k = k
        ModelSolver().simulate(m, 0.0, stoptime, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.plot(m.signals['time'], m.ddriver.signals['steering'], label='steering')
        p1.plot(m.signals['time'], m.kdriver.signals['steering'], label='steering')
        p1.legend()
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'], label='dX vs dY')
        p2.plot(m.kbike.signals['x'], m.kbike.signals['y'], label='kx vs ky')
        p2.legend()
        p3.plot(m.dbike.signals['time'], m.dbike.signals['vy'], label='dvy')
        p3.legend()
        p4.plot(m.dbike.signals['time'], m.dbike.signals['dpsi'], label='dpsi')
        p4.plot(m.kbike.signals['time'], m.kbike.signals['der_psi'], label='der_psi')
        p4.legend()
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
        self.plot_twobikes(0.0, 0.8819660112501051, 1, 25)
        # self.plot_twobikes(0.6541019662496845, 0.8819660112501051, 2, 50)

    def cost_two_bikes(self, p):
        # print("Running sim with delay={} and k={}...".format(p[0], p[1]))
        residuals = self.cost_two_bikes_lse(p)
        cost = (residuals**2).sum()
        # print("Running sim with delay={} and k={}... Done. Cost={}.".format(p[0], p[1], cost))
        return cost

    def cost_two_bikes_lse(self, p):
        delay, k = p
        m = BikeModelsWithDriver()
        m.kdriver.delay = delay
        m.kdriver.k = k
        ModelSolver().simulate(m, 0.0, 33, 0.1)
        dX = np.array(m.dbike.signals['X'])
        dY = np.array(m.dbike.signals['Y'])
        kX = np.array(m.kbike.signals['x'])
        kY = np.array(m.kbike.signals['y'])
        errx = (dX - kX)
        erry = (dY - kY)
        ssd = np.concatenate((errx, erry))
        return ssd

    def test_calibrate_kinematics_driver(self):
        def cost_d(d):
            sol_d = minimize_scalar(lambda k: self.cost_two_bikes([d, k]), method='bounded', bounds=(0.5, 1.0),
                                  options={'xatol': 0.1})
            print("d=", d)
            print("k=", sol_d.x)
            print("cost=", sol_d.fun)
            print("msg=", sol_d.message)
            return sol_d.fun

        sol = minimize_scalar(cost_d, method='bounded', bounds=(0.6, 1.2),
                        options={'xatol': 0.1})
        print(sol)


    def test_robotti_model_with_driver(self):
        m = RobottiDynamicModelWithDriver()

        m.reset()
        ModelSolver().simulate(m, 0.0, 60, 0.1)
        _, (p1, p2, p3) = plt.subplots(1, 3)

        p1.plot(m.signals['time'], m.driver.signals['steering'])
        p2.plot(m.rbike.signals['X'], m.rbike.signals['Y'])
        p3.plot(m.signals['time'], m.rbike.signals['vx'])
        plt.show()

    def test_counter_intuitive_robotti(self):
        def simulate(caf):
            m = RobottiDynamicModelWithDriver()
            m.rbike.Caf = lambda: caf
            ModelSolver().simulate(m, 0.0, 10.0, 0.1)
            return m

        c1 = 20000
        c2 = 100
        m1 = simulate(c1)
        m2 = simulate(c2)

        _, (p1, p2, p3) = plt.subplots(1, 3)
        p1.plot(m1.signals['time'], m1.driver.signals['steering'])
        p2.plot(m1.rbike.signals['X'], m1.rbike.signals['Y'], label="caf={}".format(c1))
        p2.plot(m2.rbike.signals['X'], m2.rbike.signals['Y'], label="caf={}".format(c2))
        p2.legend()
        p3.plot(m1.signals['time'], m1.rbike.signals['vy'])
        p3.plot(m2.signals['time'], m2.rbike.signals['vy'])
        plt.show()

    def test_plot_two_robotti_models(self):
        m = RobottiBikeModelsWithDriver()
        ModelSolver().simulate(m, 0.0, 30, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)
        p1.plot(m.signals['time'], m.robot.signals['Fcf'], label='robot.Fcf')
        p1.plot(m.signals['time'], m.dbike.signals['Fcf'], label='dbike.Fcf')
        p1.plot(m.signals['time'], m.ddriver.signals['steering'], label='steering')
        p1.legend()
        p2.plot(m.robot.signals['X'], m.robot.signals['Y'], label='robotti')
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'], label='bike')
        p2.legend()
        plt.show()

