from Model import Model


class KinematicsDriver(Model):

    def __init__(self):
        super().__init__()
        self.delay = self.parameter(0.3)
        self.k = self.parameter(0.9)
        self.u = self.input()
        self.steering = self.var(lambda: self.k*self.u(- self.delay))
        self.save()


