from oomodelling.Model import Model

from DriverDynamic import DriverDynamic
from RobottiDynamicModel import RobottiDynamicModel


class RobottiDynamicModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.ddriver = DriverDynamic()
        self.dbike = RobottiDynamicModel()

        self.dbike.deltaFl = self.ddriver.steering
        self.dbike.deltaFr = self.ddriver.steering

        self.dbike.vel_left = lambda: 1.0
        self.dbike.vel_right = lambda: 1.0

        self.save()
