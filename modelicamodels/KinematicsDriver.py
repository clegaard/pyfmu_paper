from Model import Model


class KinematicsDriver(Model):

    def __init__(self):
        super().__init__()
        self.parameter('delay', 0.3)
        self.parameter('k', 0.9)
        self.input('u')
        self.var('y', self.get_out)

    def get_out(self):
        self.k*self.u(- self.delay)


