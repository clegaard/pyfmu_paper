from BikeDynamicModel import BikeDynamicModel
from DynamicDriver import DynamicDriver
from Model import Model


class DynamicBikeModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.model('ddriver', DynamicDriver())
        self.model('dbike', BikeDynamicModel())

        self.connect(self.dbike, 'deltaf', self.ddriver, 'steering')

        self.save()



