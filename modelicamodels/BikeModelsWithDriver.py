from BikeDynamicModel import BikeDynamicModel
from BikeModel import BikeModel
from DynamicDriver import DynamicDriver
from KinematicsDriver import KinematicsDriver
from Model import Model


class BikeModelsWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.model('ddriver', DynamicDriver())
        self.model('dbike', BikeDynamicModel())
        self.model('kdriver', KinematicsDriver())
        self.model('kbike', BikeModel())

        self.connect(self.dbike, 'deltaf', self.ddriver, 'steering')
        self.connect(self.kdriver, 'u', self.ddriver, 'steering')
        self.connect(self.kbike, 'deltaf', self.kdriver, 'steering')

        self.save()



