import numpy as np

from BikeDynamicModel import BikeDynamicModel
from BikeModel import BikeModel
from DynamicDriver import DynamicDriver
from KinematicsDriver import KinematicsDriver
from Model import Model


class SystemToTrack(Model):

    def __init__(self):
        super().__init__()
        self.ddriver = DynamicDriver()
        self.dbike = BikeDynamicModel()

        self.dbike.deltaf = self.ddriver.steering

        self.x = self.var(self.dbike.X)
        self.y = self.var(self.dbike.Y)
        self.steering = self.var(self.ddriver.steering)

        self.save()


class TrackingModel(Model):
    def __init__(self):
        super().__init__()

        # Submodels
        self.kdriver = KinematicsDriver()
        self.kbike = BikeModel()

        # General parameters of any tracking simulator
        self.horizon = self.parameter(1.0)
        self.sample_size = self.parameter(0.1)
        self.error = self.var(self.get_error)

        self.tracked_x = self.input()
        self.tracked_y = self.input()
        self.control_steering = self.input()
        self.x = self.var(self.kbike.x)
        self.y = self.var(self.kbike.y)

        self.kdriver.u = self.control_steering
        self.kbike.deltaf = self.kdriver.steering

        self.save()

    def get_error(self):
        """
        Computes the error over the horizon.
        Discretizes the tracked signals over the horizon using the sample size given.
        :return:
        """
        max_horizon = min(self.time(), self.horizon) # We can't go beyong time 0
        error_space = np.arange(0.0, max_horizon, self.sample_size)
        tracked_x_traj = np.array([self.tracked_x(-ti) for ti in error_space])
        actual_x = np.array([self.x(-ti) for ti in error_space])
        tracked_y_traj = np.array([self.tracked_y(-ti) for ti in error_space])
        actual_y = np.array([self.y(-ti) for ti in error_space])
        error = ((tracked_x_traj-actual_x)**2).sum() + ((tracked_y_traj-actual_y)**2).sum()
        return error


class TrackingSimulator(Model):
    def __init__(self):
        super().__init__()

        self.to_track = SystemToTrack()
        self.tracking = TrackingModel()

        self.tracking.tracked_x = self.to_track.x
        self.tracking.tracked_y = self.to_track.y
        self.tracking.control_steering = self.to_track.steering

        self.save()


