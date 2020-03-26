import unittest
import matplotlib.pyplot as plt

from cosimlibrary.jacobi_runner import JacobiRunner
from cosimlibrary.scenario import OutputConnection, VarType, SignalType, CosimScenario, Connection
from fmpy import *

from cosimlibrary.loader import FMULoader

from virtual_driver import VirtualDriver
from virtual_robotti import VirtualRobotti
from virtual_tracking_simulator import VirtualTrackingRobotti


class MyTestCase(unittest.TestCase):

    def test_dump(self):
        dump('TrackingSimulator.fmu')

    def test_load(self):
        def logger(msg):
            print(msg)

        tracking = FMULoader.load("TrackingSimulator.fmu", "tracking", logger)
        tracking.fmu.instantiate(loggingOn=True)

        # TODO Define the steering input and robotti as virtual FMUs and run the cosim.

        # print(tracking.vars)
        # self.assertTrue('steering' in tracking.vars)
        # FMULoader.unload(tracking)
        #
        # robotti = FMULoader.load("robotti_global.fmu", "robotti", logger)
        # robotti.fmu.instantiate(loggingOn=True)
        # # print(robotti.vars)
        # self.assertTrue('Demux.y1' in robotti.vars)
        # FMULoader.unload(robotti)
        #
        # steering = FMULoader.load("Steering_input.fmu", "steering", logger)
        # steering.fmu.instantiate(loggingOn=True)
        # # print(steering.vars)
        # self.assertTrue('output_v' in steering.vars)
        # FMULoader.unload(steering)

    def test_run_driver(self):
        def logger(msg):
            # print(msg)
            pass

        driver = VirtualDriver('driver')
        steering_out = OutputConnection(value_type=VarType.REAL,
                                    signal_type=SignalType.CONTINUOUS,
                                    source_fmu=driver,
                                    source_vr=[
                                        driver.steering
                                    ])
        output_connections = [steering_out]

        real_parameters = {}

        scenario = CosimScenario(
            fmus=[driver],
            connections=[],
            step_size=0.01,
            print_interval=0.1,
            stop_time=60.0,
            outputs=output_connections,
            real_parameters=real_parameters)

        driver.instantiate(loggingOn=False)
        jacobi = JacobiRunner()
        results = jacobi.run_cosim(scenario, lambda t: print(t))
        driver.terminate()

        plt.figure(1)
        plt.xlabel("time")
        plt.plot(results.timestamps, results.signals[driver.instanceName][driver.steering], label="output_v")
        plt.legend()

        plt.show()

    def test_run_robotti(self):
        def logger(msg):
            # print(msg)
            pass

        steering = VirtualDriver('steering')
        robotti = VirtualRobotti('robot')

        steering_robotti = Connection(value_type=VarType.REAL,
                              signal_type=SignalType.CONTINUOUS,
                              source_fmu=steering,
                              target_fmu=robotti,
                              source_vr=[steering.steering],
                              target_vr=[robotti.steering])

        robotti_out = OutputConnection(value_type=VarType.REAL,
                                        signal_type=SignalType.CONTINUOUS,
                                        source_fmu=robotti,
                                        source_vr=[
                                            robotti.X,
                                            robotti.Y,
                                            robotti.vx,
                                            robotti.steering,
                                        ])

        connections = [steering_robotti]
        output_connections = [steering_robotti, robotti_out]

        real_parameters = {}

        stop_time = 60.0

        scenario = CosimScenario(
            fmus=[steering, robotti],
            connections=connections,
            step_size=0.01,
            print_interval=0.1,
            stop_time=stop_time,
            outputs=output_connections,
            real_parameters=real_parameters)

        steering.instantiate(loggingOn=False)
        robotti.instantiate(loggingOn=False)

        jacobi = JacobiRunner()
        results = jacobi.run_cosim(scenario, lambda t: print(t))
        steering.terminate()

        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(results.timestamps, results.signals[steering.instanceName][steering.steering], label="dsteering")
        p1.plot(results.timestamps, results.signals[robotti.instanceName][robotti.steering], label="rsteering")
        p1.plot(results.timestamps, [robotti.model.deltaFl(-(stop_time-t)) for t in results.timestamps], label="delayed_track_steering_left")
        p1.plot(results.timestamps, [robotti.model.deltaFr(-(stop_time-t)) for t in results.timestamps], label="delayed_track_steering_right")
        p1.legend()
        p2.plot(results.signals[robotti.instanceName][robotti.X], results.signals[robotti.instanceName][robotti.Y], label='X vs Y')
        p2.legend()
        plt.show()

    def test_run_tracking_robotti(self):
        def logger(msg):
            print(msg)

        steering = VirtualDriver('steering')
        robotti = VirtualRobotti('robot')
        tracking = FMULoader.load("TrackingSimulator.fmu", "tracking", logger)

        steering_robotti = Connection(value_type=VarType.REAL,
                              signal_type=SignalType.CONTINUOUS,
                              source_fmu=steering,
                              target_fmu=robotti,
                              source_vr=[steering.steering],
                              target_vr=[robotti.steering])

        steering_tracking = Connection(value_type=VarType.REAL,
                                       signal_type=SignalType.CONTINUOUS,
                                       source_fmu=steering,
                                       target_fmu=tracking.fmu,
                                       source_vr=[steering.steering],
                                       target_vr=[tracking.vars['steering'].valueReference])

        robotti_tracking = Connection(value_type=VarType.REAL,
                                       signal_type=SignalType.CONTINUOUS,
                                       source_fmu=robotti,
                                       target_fmu=tracking.fmu,
                                       source_vr=[robotti.X, robotti.Y, robotti.vx],
                                       target_vr=[tracking.vars['X'].valueReference, tracking.vars['Y'].valueReference, tracking.vars['vx'].valueReference])

        tracking_out = OutputConnection(value_type=VarType.REAL,
                                       signal_type=SignalType.CONTINUOUS,
                                       source_fmu=tracking.fmu,
                                       source_vr=[
                                           tracking.vars['tracking_X'].valueReference,
                                           tracking.vars['tracking_Y'].valueReference
                                       ])

        connections = [steering_robotti, steering_tracking, robotti_tracking]
        output_connections = [steering_tracking, robotti_tracking, tracking_out]
        fmus = [steering, robotti, tracking.fmu]

        real_parameters = {}

        scenario = CosimScenario(
            fmus=fmus,
            connections=connections,
            step_size=0.01,
            print_interval=0.01,
            stop_time=30.0,
            outputs=output_connections,
            real_parameters=real_parameters)

        steering.instantiate(loggingOn=False)
        robotti.instantiate(loggingOn=False)
        tracking.fmu.instantiate(loggingOn=True)

        jacobi = JacobiRunner()
        results = jacobi.run_cosim(scenario, lambda t: print(t))
        steering.terminate()
        robotti.terminate()
        tracking.fmu.terminate()
        FMULoader.unload(tracking)

        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(results.timestamps, results.signals[steering.instanceName][steering.steering], label="steering")
        p1.legend()
        p2.plot(results.signals[robotti.instanceName][robotti.X], results.signals[robotti.instanceName][robotti.Y], 'o', label='X vs Y')
        p2.plot(results.signals[tracking.fmu.instanceName][tracking.vars['tracking_X'].valueReference], results.signals[tracking.fmu.instanceName][tracking.vars['tracking_Y'].valueReference], 'o', label='~X vs ~Y')
        p2.legend()
        plt.show()

    def test_run_virtual_tracking_robotti(self):
        def logger(msg):
            print(msg)

        steering = VirtualDriver('steering')
        robotti = VirtualRobotti('robot')
        tracking = VirtualTrackingRobotti("tracking")

        steering_robotti = Connection(value_type=VarType.REAL,
                              signal_type=SignalType.CONTINUOUS,
                              source_fmu=steering,
                              target_fmu=robotti,
                              source_vr=[steering.steering],
                              target_vr=[robotti.steering])

        steering_tracking = Connection(value_type=VarType.REAL,
                                       signal_type=SignalType.CONTINUOUS,
                                       source_fmu=steering,
                                       target_fmu=tracking,
                                       source_vr=[steering.steering],
                                       target_vr=[tracking.steering])

        robotti_tracking = Connection(value_type=VarType.REAL,
                                       signal_type=SignalType.CONTINUOUS,
                                       source_fmu=robotti,
                                       target_fmu=tracking,
                                       source_vr=[robotti.X, robotti.Y, robotti.vx],
                                       target_vr=[tracking.X, tracking.Y, tracking.vx])

        tracking_out = OutputConnection(value_type=VarType.REAL,
                                       signal_type=SignalType.CONTINUOUS,
                                       source_fmu=tracking,
                                       source_vr=[
                                           tracking.tracking_X,
                                           tracking.tracking_Y,
                                           tracking.steering
                                       ])

        connections = [steering_robotti, steering_tracking, robotti_tracking]
        output_connections = [steering_tracking, robotti_tracking, tracking_out]
        fmus = [steering, robotti, tracking]

        real_parameters = {}
        stop_time = 15.0
        scenario = CosimScenario(
            fmus=fmus,
            connections=connections,
            step_size=0.01,
            print_interval=0.1,
            stop_time=stop_time,
            outputs=output_connections,
            real_parameters=real_parameters)

        steering.instantiate(loggingOn=False)
        robotti.instantiate(loggingOn=False)
        tracking.instantiate(loggingOn=True)

        jacobi = JacobiRunner()
        results = jacobi.run_cosim(scenario, lambda t: print(t))
        steering.terminate()
        robotti.terminate()
        tracking.terminate()

        _, (p1, p2) = plt.subplots(1, 2)

        p1.plot(results.timestamps, results.signals[steering.instanceName][steering.steering], label="steering")
        p1.plot(results.timestamps, results.signals[tracking.instanceName][tracking.steering], label="track_steering")
        p1.plot(results.timestamps, [tracking.model.steering(-(stop_time-t)) for t in results.timestamps], label="delayed_track_steering")
        p1.legend()
        p2.plot(results.signals[robotti.instanceName][robotti.X], results.signals[robotti.instanceName][robotti.Y], 'o', label='X vs Y')
        p2.plot(results.signals[tracking.instanceName][tracking.tracking_X], results.signals[tracking.instanceName][tracking.tracking_Y], 'o', label='~X vs ~Y')
        p2.legend()
        plt.show()



if __name__ == '__main__':
    unittest.main()
