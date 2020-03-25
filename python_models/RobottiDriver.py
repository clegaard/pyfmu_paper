import math

from oomodelling.Model import Model


class RobottiDriver(Model):

    def __init__(self):
        super().__init__()
        self.steering = self.var(lambda: 0.5*math.sin((math.pi/10.0)*self.time()))
        self.save()

