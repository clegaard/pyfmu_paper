from BikeModel import BikeModel
from DynamicDriver import DynamicDriver
from Model import Model


class BikeModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.model('driver', DynamicDriver())
        self.model('bike', BikeModel())

        self.connect(self.bike, 'deltaf', self.driver, 'steering')

        self.save()


