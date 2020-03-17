from BikeDynamicModel import BikeDynamicModel
from DriverDynamic import DriverDynamic
from Model import Model


class BikeDynamicModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.ddriver = DriverDynamic()
        self.dbike = BikeDynamicModel()

        self.dbike.deltaf = self.ddriver.steering

        self.save()



