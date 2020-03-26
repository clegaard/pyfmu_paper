import math

from cosimlibrary.virtual_fmus import VirtualFMU
from fmpy.fmi2 import fmi2True, fmi2OK
from oomodelling.Model import Model


# TODO This can be generalized: produce a factory that takes a Model and produces a VirtualFMU of that model.
# For now, due to time constraints, we duplicate the code.
from oomodelling.ModelSolver import StepRK45
from scipy.integrate import solve_ivp
import numpy as np

from trackingsimulatorproject import RobottiTrackingSimulator


class VirtualDriver(VirtualFMU):
    def __init__(self, instanceName):
        ref = 0
        self.steering = ref
        ref += 1
        self.X = ref
        ref += 1
        self.Y = ref
        ref += 1
        self.vx = ref
        ref += 1

        # Internal model
        self.model = RobottiTrackingSimulator()

        self.model.dbike.deltaf = lambda: self.steering
        self.model.steering = lambda: self.steering
        self.model.to_track_X = lambda: self.X
        self.model.to_track_Y = lambda: self.Y
        self.model.to_track_vx = lambda: self.vx
        self.model.dbike.vx = lambda: self.vx

        self.model.tolerance = 0.1
        self.model.horizon = 5.0
        self.model.max_iterations = 10
        self.model.cooldown = 5.0
        self.model.nsamples = 20
        self.model.time_step = 0.1
        self.model.conv_xatol = 30.0
        self.model.conv_fatol = 1.0

        self.tracking_X = ref
        ref += 1
        self.tracking_Y = ref
        ref += 1



        super().__init__(instanceName, ref)

    def reset(self):
        super().reset()
        self.state[self.steering] = 0.0
        self.driver.set_time(self.start_time)

    def setupExperiment(self, tolerance=None, startTime=0.0, stopTime=None):
        super().setupExperiment(tolerance, startTime, stopTime)
        self.start_time = startTime
        self.driver.set_time(startTime)

    def exitInitializationMode(self):
        super().exitInitializationMode()
        self.driver.assert_initialized()
        x = self.driver.state_vector()
        self.driver.record_state(x, self.start_time)

    def doStep(self, currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPoint=fmi2True):
        f = self.driver.derivatives()
        x = self.driver.state_vector()
        sol = solve_ivp(f, (currentCommunicationPoint, currentCommunicationPoint + communicationStepSize), x, method=StepRK45, max_step=communicationStepSize,
                        model=self.driver, t_eval=[currentCommunicationPoint + communicationStepSize])
        assert sol.success
        assert np.isclose(self.driver.time(), currentCommunicationPoint + communicationStepSize)

        self.state[self.steering] = self.driver.steering()

        return fmi2OK