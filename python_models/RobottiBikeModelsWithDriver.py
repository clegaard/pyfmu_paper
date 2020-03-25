from BikeDynamicModel import BikeDynamicModel
from BikeDynamicModelSpeedDriven import BikeDynamicModelSpeedDriven
from BikeKinematicModel import BikeKinematicModel
from DriverDynamic import DriverDynamic
from DriverKinematic import DriverKinematic
from oomodelling.Model import Model

from RobottiDriver import RobottiDriver
from RobottiDynamicModel import RobottiDynamicModel


class RobottiBikeModelsWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.ddriver = RobottiDriver()
        self.dbike = BikeDynamicModelSpeedDriven()
        self.robot = RobottiDynamicModel()

        self.dbike.m = self.robot.m
        self.dbike.lf = self.robot.lf
        self.dbike.lr = self.robot.lr
        self.dbike.Iz = self.robot.Iz
        self.dbike.vx = self.robot.vx

        self.dbike.Caf = lambda: self.robot.Car
        self.dbike.Car = self.robot.Car

        self.dbike.deltaf = self.ddriver.steering
        self.robot.deltaFl = self.ddriver.steering
        self.robot.deltaFr = self.ddriver.steering

        self.save()



