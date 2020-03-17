import unittest
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares, minimize_scalar

from BikeKinematicModel import BikeKinematicModel
from BikeKinematicModelWithDriver import BikeKinematicModelWithDriver
from BikeModelsWithDriver import BikeModelsWithDriver
from BikeDynamicModelWithDriver import BikeDynamicModelWithDriver
from DriverDynamic import DriverDynamic
from ModelSolver import ModelSolver
from BikeTrackingWithKinematic import TrackingSimulator, BikeTrackingSimulator


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

        m.reset()
        ModelSolver().simulate(m, 0.0, 10.0, 0.1)
        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(m.signals['time'], m.ddriver.signals['steering'])
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'])
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

    def test_tracking_model(self):
        m = BikeTrackingSimulator()
        m.tolerance = 10
        m.horizon = 5.0
        m.nsamples = 20
        m.time_step = 0.1
        m.tracking.kdriver.delay = 0.6541019662496845
        m.tracking.kdriver.k = 0.8819660112501051
        m.to_track.ddriver.nperiods = 2

        ModelSolver().simulate(m, 0.0, 30, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.plot(m.signals['time'], m.to_track.ddriver.signals['steering'], label='steering')
        p1.plot(m.signals['time'], m.tracking.kdriver.signals['steering'], label='steering')
        p1.legend()
        p2.plot(m.to_track.dbike.signals['X'], m.to_track.dbike.signals['Y'], label='dX vs dY')
        p2.plot(m.tracking.kbike.signals['x'], m.tracking.kbike.signals['y'], label='kx vs ky')
        for calib in m.recalibration_history:
            p2.plot(calib.xs[3, :], calib.xs[4, :], '--', label='recalibration')
        p2.legend()
        p3.plot(m.to_track.dbike.signals['time'], m.to_track.dbike.signals['Y'], label='dy')
        p3.plot(m.tracking.kbike.signals['time'], m.tracking.kbike.signals['y'], label='ky')
        p3.legend()
        p4.plot(m.signals['time'], m.signals['error'], label='error')
        p4.legend()
        plt.show()
