import unittest
import matplotlib.pyplot as plt

from cosimlibrary.jacobi_runner import JacobiRunner
from cosimlibrary.scenario import OutputConnection, VarType, SignalType, CosimScenario, Connection
from fmpy import *

from cosimlibrary.loader import FMULoader

from virtual_driver import VirtualDriver


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

        steering = FMULoader.load("Steering_input.fmu", "steering", logger)
        steering_out = OutputConnection(value_type=VarType.REAL,
                                    signal_type=SignalType.CONTINUOUS,
                                    source_fmu=steering.fmu,
                                    source_vr=[
                                        steering.vars['output_v'].valueReference,
                                        steering.vars['output_R_t'].valueReference,
                                        steering.vars['output_delta_FL'].valueReference,
                                        steering.vars['output_delta_FR'].valueReference,
                                        steering.vars['output_deltaR'].valueReference,
                                        steering.vars['output_omega'].valueReference
                                    ])
        output_connections = [steering_out]

        real_parameters = {}

        scenario = CosimScenario(
            fmus=[steering.fmu],
            connections=output_connections,
            step_size=0.01,
            print_interval=0.1,
            stop_time=60.0,
            outputs=output_connections,
            real_parameters=real_parameters)

        steering.fmu.instantiate(loggingOn=False)
        jacobi = JacobiRunner()
        results = jacobi.run_cosim(scenario, lambda t: print(t))
        steering.fmu.terminate()
        FMULoader.unload(steering)

        plt.figure(1)
        plt.xlabel("time")
        plt.plot(results.timestamps, results.signals[steering.fmu.instanceName][steering.vars['output_v'].valueReference], label="output_v")
        # plt.plot(results.timestamps, results.signals[steering.fmu.instanceName][steering.vars['output_R_t'].valueReference], label="output_R_t")
        plt.plot(results.timestamps, results.signals[steering.fmu.instanceName][steering.vars['output_delta_FL'].valueReference], label="output_delta_FL")
        plt.plot(results.timestamps, results.signals[steering.fmu.instanceName][steering.vars['output_delta_FR'].valueReference], label="output_delta_FR")
        plt.plot(results.timestamps, results.signals[steering.fmu.instanceName][steering.vars['output_deltaR'].valueReference], label="output_deltaR")
        plt.plot(results.timestamps, results.signals[steering.fmu.instanceName][steering.vars['output_omega'].valueReference], label="output_omega")
        plt.legend()

        plt.show()

    def test_run_robotti(self):
        def logger(msg):
            # print(msg)
            pass

        steering = VirtualDriver('steering')
        robotti = FMULoader.load("robotti_global.fmu", "robotti", logger)

        steering_robotti = Connection(value_type=VarType.REAL,
                              signal_type=SignalType.CONTINUOUS,
                              source_fmu=steering,
                              target_fmu=robotti.fmu,
                              source_vr=[steering.v],
                              target_vr=[robotti.vars['F_x_in'].valueReference])

        steering_out = OutputConnection(value_type=VarType.REAL,
                                    signal_type=SignalType.CONTINUOUS,
                                    source_fmu=steering,
                                    source_vr=[
                                        steering.v,
                                    ])
        robotti_out = OutputConnection(value_type=VarType.REAL,
                                        signal_type=SignalType.CONTINUOUS,
                                        source_fmu=robotti.fmu,
                                        source_vr=[
                                            robotti.vars['y3_out'].valueReference,
                                            robotti.vars['y4_out'].valueReference,
                                            robotti.vars['y5_out'].valueReference,
                                        ])

        connections = [steering_robotti]
        output_connections = [steering_out, robotti_out]

        real_parameters = {}

        scenario = CosimScenario(
            fmus=[steering],
            connections=connections,
            step_size=0.01,
            print_interval=0.1,
            stop_time=60.0,
            outputs=output_connections,
            real_parameters=real_parameters)

        steering.instantiate(loggingOn=False)
        robotti.fmu.instantiate(loggingOn=False)

        jacobi = JacobiRunner()
        results = jacobi.run_cosim(scenario, lambda t: print(t))
        steering.terminate()

        plt.figure(1)
        plt.xlabel("time")
        plt.plot(results.timestamps, results.signals[steering.instanceName][steering.v], label="output_v")
        plt.plot(results.timestamps, results.signals[robotti.fmu.instanceName][robotti.vars['y3_out'].valueReference], label="y3_out")
        plt.plot(results.timestamps, results.signals[robotti.fmu.instanceName][robotti.vars['y4_out'].valueReference], label="y4_out")
        plt.plot(results.timestamps, results.signals[robotti.fmu.instanceName][robotti.vars['y5_out'].valueReference], label="y5_out")
        plt.legend()

        plt.show()




if __name__ == '__main__':
    unittest.main()
