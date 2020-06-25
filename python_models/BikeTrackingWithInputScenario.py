from random import random

import numpy as np
from oomodelling.Model import Model

from BikeDynamicModel import BikeDynamicModel
from BikeDynamicModelWithDriver import BikeDynamicModelWithDriver
from oomodelling.ModelSolver import ModelSolver
from oomodelling.TrackingSimulator import TrackingSimulator

from BikeTrackingWithInput import BikeTrackingWithInput


class BikeTrackingWithInputScenario(Model):
    def __init__(self):
        super().__init__()

        self.to_track = BikeDynamicModelWithDriver()
        self.tracking = BikeTrackingWithInput()

        self._rand_Caf = 800

        self.to_track.dbike.Caf = lambda: self._rand_Caf

        self.tracking.to_track_X = self.to_track.dbike.X
        self.tracking.to_track_Y = self.to_track.dbike.Y
        self.tracking.to_track_delta = self.to_track.ddriver.steering

        self.save()

    def update_Caf(self):
        return 800 if self.time() < 10.0 else 500

    def discrete_step(self):
        super().discrete_step()
        self._rand_Caf = self.update_Caf()
        return True
