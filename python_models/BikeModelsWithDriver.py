from BikeDynamicModel import BikeDynamicModel
from BikeKinematicModel import BikeKinematicModel
from DriverDynamic import DriverDynamic
from DriverKinematic import DriverKinematic
from oomodelling.Model import Model


class BikeModelsWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.ddriver = DriverDynamic()
        self.dbike = BikeDynamicModel()
        self.kdriver = DriverKinematic()
        self.kbike = BikeKinematicModel()

        self.dbike.deltaf = self.ddriver.steering
        self.kdriver.u = self.ddriver.steering
        self.kbike.deltaf = self.kdriver.steering

        self.save()



