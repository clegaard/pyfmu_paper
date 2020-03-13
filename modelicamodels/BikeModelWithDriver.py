from BikeModel import BikeModel
from Driver import Driver
from Model import Model


class BikeModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.model('driver', Driver())
        self.model('bike', BikeModel())

        self.connect(self.bike, 'deltaf', self.driver, 'steering')



