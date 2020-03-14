from BikeDynamicModel import BikeDynamicModel
from BikeModel import BikeModel
from DynamicDriver import DynamicDriver
from KinematicsDriver import KinematicsDriver
from Model import Model


class BikeModelsWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.ddriver = DynamicDriver()
        self.dbike = BikeDynamicModel()
        self.kdriver = KinematicsDriver()
        self.kbike = BikeModel()

        self.connect(self.dbike, 'deltaf', self.ddriver, 'steering')
        self.connect(self.kdriver, 'u', self.ddriver, 'steering')
        self.connect(self.kbike, 'deltaf', self.kdriver, 'steering')

        self.save()



