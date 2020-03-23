from BikeKinematicModel import BikeKinematicModel
from DriverDynamic import DriverDynamic
from oomodelling.Model import Model


class BikeKinematicModelWithDriver(Model):

    def __init__(self):
        super().__init__()

        self.driver = DriverDynamic()
        self.bike = BikeKinematicModel()

        self.bike.deltaf = self.driver.steering

        self.save()


