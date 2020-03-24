from cosimlibrary.virtual_fmus import VirtualFMU
from fmpy.fmi2 import fmi2True


class VirtualDriver(VirtualFMU):
    def __init__(self, instanceName):
        ref = 0
        self.v = ref;
        ref += 1

        super().__init__(instanceName, ref)

    def reset(self):
        super().reset()
        self.state[self.v] = 1.0

    def doStep(self, currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPoint=fmi2True):
        self.state[self.v] = 1.0
