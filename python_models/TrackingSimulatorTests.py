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
from BikeTrackingWithInputScenario import BikeTrackingWithInputScenario
from DriverDynamic import DriverDynamic
from oomodelling.ModelSolver import ModelSolver
from BikeTrackingWithKinematic import TrackingSimulator, BikeTrackingSimulatorKinematic
from RobottiBikeModelsWithDriver import RobottiBikeModelsWithDriver
from RobottiDynamicModelWithDriver import RobottiDynamicModelWithDriver
from RobottiTrackingSimulator import RobottiTrackingSimulator
from RobottiTrackingSimulatorRandomNoise import RobottiTrackingSimulatorRandomNoise

class TrackingSimulatorTests(unittest.TestCase):

    def test_tracking_kinematic(self):
        m = BikeTrackingSimulatorKinematic()
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

    def test_tracking_dynamic(self):
        seed(1)
        m = BikeTrackingSimulatorDynamic()
        # Bump the Caf after some time
        # m.to_track.dbike.Caf = lambda: 800 if m.time() < 13.0 else 500
        # m.to_track.dbike.Caf = lambda: 800
        m.tolerance = 0.02
        m.horizon = 5.0
        m.cooldown = 5.0
        m.nsamples = 10
        m.time_step = 0.1
        m.conv_xatol = 30.0
        m.conv_fatol = 1.0

        m.to_track.ddriver.nperiods = 2

        ModelSolver().simulate(m, 0.0, 60.0, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.scatter(m.signals['time'], m.to_track.ddriver.signals['steering'], c=m.signals['time'], s=1.0, label='steering')
        p1.legend()
        p2.scatter(m.to_track.dbike.signals['X'], m.to_track.dbike.signals['Y'], c=m.signals['time'], s=1.0, label='dX vs dY')
        p2.plot(m.tracking.signals['X'], m.tracking.signals['Y'], label='~dX vs ~dY')
        for calib in m.recalibration_history:
            p2.plot(calib.xs[m.X_idx, :], calib.xs[m.Y_idx, :], '--', label='recalibration')
        p2.legend()
        p3.plot(m.signals['time'], m.signals['error'], label='error')
        p3.plot(m.signals['time'], [m.tolerance for t in m.signals['time']], label='tolerance')
        p3.legend()
        p4.plot(m.to_track.dbike.signals['time'], m.to_track.dbike.signals['Caf'], label='real_Caf')
        p4.plot(m.tracking.signals['time'], m.tracking.signals['Caf'], label='approx_Caf')
        p4.legend()
        plt.show()


    def test_generate_paper_figure(self):
        m = BikeTrackingWithDynamicWithoutStateRestore()
        m.tolerance = 0.2
        m.horizon = 5.0
        m.cooldown = 5.0
        m.nsamples = 10
        m.max_iterations = 20
        m.time_step = 0.1
        m.conv_xatol = 1e3
        m.conv_fatol = 0.01

        m.to_track.ddriver.nperiods = 2

        ModelSolver().simulate(m, 0.0, 40.0, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.plot(m.signals['time'], m.to_track.ddriver.signals['steering'],
                   label='steering')
        p1.legend()
        p2.plot(m.to_track.dbike.signals['X'], m.to_track.dbike.signals['Y'],
                   label='dX vs dY')
        p2.plot(m.tracking.signals['X'], m.tracking.signals['Y'], label='~dX vs ~dY')
        for calib in m.recalibration_history:
            p2.plot(calib.xs[m.X_idx, :], calib.xs[m.Y_idx, :], '--', label='recalibration')
        p2.legend()
        p3.plot(m.signals['time'], m.signals['error'], label='error')
        p3.plot(m.signals['time'], [m.tolerance for t in m.signals['time']], label='tolerance')
        p3.legend()
        p4.plot(m.to_track.dbike.signals['time'], m.to_track.dbike.signals['Caf'], label='real_Caf')
        p4.plot(m.tracking.signals['time'], m.tracking.signals['Caf'], label='approx_Caf')
        p4.legend()
        plt.show()

    def test_robotti_tracking(self):
        m = RobottiTrackingSimulator()
        # Bump the Caf after some time
        m.robot.Caf = lambda: 20000 if m.time() < 5.0 else 20000
        m.tolerance = 0.1
        m.horizon = 2.0
        m.max_iterations = 20
        m.cooldown = 5.0
        m.nsamples = 20
        m.time_step = 0.1
        m.conv_xatol = 0.1
        m.conv_fatol = 0.1

        print(m.robot.m)
        print(m.robot.lf)
        print(m.robot.lr)
        print(m.robot.Iz)
        print(m.robot.Car)

        stop_time = 15.0
        ModelSolver().simulate(m, 0.0, stop_time, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.plot(m.signals['time'], m.driver.signals['steering'], label='steering')
        p1.plot(m.signals['time'], [m.driver.steering(-(stop_time - t)) for t in m.signals['time']], label='steering')
        p1.legend()
        p2.plot(m.robot.signals['X'], m.robot.signals['Y'], label='dX vs dY')
        p2.plot(m.dbike.signals['X'], m.dbike.signals['Y'], label='~dX vs ~dY')
        for calib in m.recalibration_history:
            p2.plot(calib.xs[m.X_idx, :], calib.xs[m.Y_idx, :], '--', label='recalibration')
        p2.legend()
        p3.plot(m.signals['time'], m.signals['error'], label='error')
        p3.plot(m.signals['time'], [m.tolerance for t in m.signals['time']], label='tolerance')
        p3.legend()
        # p4.plot(m.robot.signals['time'], m.robot.signals['Caf'], label='real_Caf')
        p4.plot(m.dbike.signals['time'], m.dbike.signals['Caf'], label='approx_Caf')
        p4.legend()
        plt.show()


    def test_robotti_tracking_randomnoise(self):
        m = RobottiTrackingSimulatorRandomNoise()
        m.driver.width = 8.0
        m.driver.nperiods = 2
        # Bump the Caf after some time
        # m.robot.Caf = lambda: max(1000, 20000 - 1000*m.time())
        m.robot.Caf = lambda: 200000
        m.tolerance = 1e10  # 0.002
        m.horizon = 20.0
        m.max_iterations = 20
        m.cooldown = 10.0
        m.nsamples = 20
        m.time_step = 0.1
        m.conv_xatol = 1e3
        m.conv_fatol = 0.1

        stop_time = 60.0
        ModelSolver().simulate(m, 0.0, stop_time, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.plot(m.signals['time'], m.driver.signals['steering'], label='steering')
        p1.plot(m.signals['time'], [m.driver.steering(-(stop_time - t)) for t in m.signals['time']], label='steering')
        p1.legend()
        p2.plot(m.robot.signals['X'], m.robot.signals['Y'], label='dX vs dY')
        p2.plot(m.tracking.signals['X'], m.tracking.signals['Y'], label='~dX vs ~dY')
        for calib in m.recalibration_history:
            p2.plot(calib.xs[m.X_idx, :], calib.xs[m.Y_idx, :], '--', label='recalibration')
        p2.legend()
        p3.plot(m.signals['time'], m.signals['error'], label='error')
        p3.plot(m.signals['time'], [m.tolerance for t in m.signals['time']], label='tolerance')
        p3.legend()
        p4.plot(m.robot.signals['time'], m.robot.signals['Caf'], label='real_Caf')
        p4.plot(m.tracking.signals['time'], m.tracking.signals['Caf'], label='approx_Caf')
        p4.legend()
        plt.show()

    def test_tracking_simulation_with_input(self):
        # logging.basicConfig(filename="tracking_with_input.log", filemode='w', level=logging.INFO)

        m = BikeTrackingWithInputScenario()
        m.tracking.tolerance = 0.2
        m.tracking.horizon = 5.0
        m.tracking.cooldown = 5.0
        m.tracking.nsamples = 10
        m.tracking.max_iterations = 20
        m.tracking.time_step = 0.1
        m.tracking.conv_xatol = 1e3
        m.tracking.conv_fatol = 0.01

        m.to_track.ddriver.nperiods = 2

        ModelSolver().simulate(m, 0.0, 25.0, 0.1)
        _, (p1, p2, p3, p4) = plt.subplots(1, 4)

        p1.plot(m.signals['time'], m.to_track.ddriver.signals['steering'],
                   label='steering')
        p1.plot(m.signals['time'], m.tracking.signals['to_track_delta'],
                label='to_track_delta')
        p1.plot(m.signals['time'], m.tracking.tracking.signals['deltaf'],
                label='deltaf')
        p1.legend()
        p2.plot(m.to_track.dbike.signals['X'], m.to_track.dbike.signals['Y'],
                   label='dX vs dY')
        p2.plot(m.tracking.tracking.signals['X'], m.tracking.tracking.signals['Y'], label='~dX vs ~dY')
        for calib in m.tracking.recalibration_history:
            p2.plot(calib.xs[m.tracking.X_idx, :], calib.xs[m.tracking.Y_idx, :], '--', label='recalibration')
        p2.legend()
        p3.plot(m.tracking.signals['time'], m.tracking.signals['error'], label='error')
        p3.plot(m.tracking.signals['time'], [m.tracking.tolerance for t in m.signals['time']], label='tolerance')
        p3.legend()
        p4.plot(m.to_track.dbike.signals['time'], m.to_track.dbike.signals['Caf'], label='real_Caf')
        p4.plot(m.tracking.tracking.signals['time'], m.tracking.tracking.signals['Caf'], label='approx_Caf')
        p4.legend()
        plt.show()


