from BikeDynamicModel import BikeDynamicModel
from BikeModel import BikeModel
from Driver import Driver
from Model import Model


class BikeModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.model('driver', Driver())
        self.model('kbike', BikeModel())
        self.model('dbike', BikeDynamicModel())

        self.connect(self.bike, 'deltaf', self.driver, 'steering')



