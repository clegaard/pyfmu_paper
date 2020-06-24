from oomodelling.Model import Model

from DriverDynamic import DriverDynamic
from RobottiDriver import RobottiDriver
from RobottiDynamicModel import RobottiDynamicModel


class RobottiDynamicModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.driver = DriverDynamic()
        self.rbike = RobottiDynamicModel()

        self.rbike.deltaFl = self.driver.steering
        self.rbike.deltaFr = self.driver.steering

        self.rbike.vel_left = lambda: 1.0
        self.rbike.vel_right = lambda: 1.0

        self.save()
