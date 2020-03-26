import unittest
import matplotlib.pyplot as plt

from cosimlibrary.jacobi_runner import JacobiRunner
from cosimlibrary.scenario import OutputConnection, VarType, SignalType, CosimScenario, Connection
from fmpy import *

from cosimlibrary.loader import FMULoader

from virtual_driver import VirtualDriver
from virtual_robotti import VirtualRobotti


from pathlib import Path


def test_run_tracking_robotti():
    def logger(msg):
        print(msg)

    p = Path(__file__).parent / "TrackingSimulator.fmus"

    steering = VirtualDriver('steering')
    robotti = VirtualRobotti('robot')
    tracking = FMULoader.load(str(p), "tracking", logger)

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
                                  source_vr=[robotti.X,
                                             robotti.Y, robotti.vx],
                                  target_vr=[tracking.vars['X'].valueReference, tracking.vars['Y'].valueReference, tracking.vars['vx'].valueReference])

    tracking_out = OutputConnection(value_type=VarType.REAL,
                                    signal_type=SignalType.CONTINUOUS,
                                    source_fmu=tracking.fmu,
                                    source_vr=[
                                        tracking.vars['tracking_X'].valueReference,
                                        tracking.vars['tracking_Y'].valueReference
                                    ])

    connections = [steering_robotti, steering_tracking, robotti_tracking]
    output_connections = [steering_tracking,
                          robotti_tracking, tracking_out]
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

    p1.plot(results.timestamps,
            results.signals[steering.instanceName][steering.steering], label="steering")
    p1.legend()
    p2.plot(results.signals[robotti.instanceName][robotti.X],
            results.signals[robotti.instanceName][robotti.Y], 'o', label='X vs Y')
    p2.plot(results.signals[tracking.fmu.instanceName][tracking.vars['tracking_X'].valueReference],
            results.signals[tracking.fmu.instanceName][tracking.vars['tracking_Y'].valueReference], 'o', label='~X vs ~Y')
    p2.legend()
    plt.show()


if __name__ == "__main__":
    test_run_tracking_robotti()
