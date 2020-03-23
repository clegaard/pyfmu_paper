import math

from oomodelling.Model import Model


class BikeKinematicModel(Model):

    def __init__(self):
        super().__init__()
        # Taken from https://ieeexplore.ieee.org/abstract/document/7225830
        # Parameters are taken from there as well.
        self.lf = self.parameter(1.105)  # distance from the the center of mass to the front (m)";
        self.lr = self.parameter(1.738)  # distance from the the center of mass to the rear (m)";
        self.x = self.state(0.0)  # longitudinal displacement in the body frame";
        self.y = self.state(0.0)  # lateral displacement in the body frame";
        self.v = self.state(1.0)  # longitudinal v";
        self.psi = self.state(0.0)  # Yaw";
        self.a = self.input(lambda: 0.0)  # longitudinal acceleration";
        self.deltaf = self.input(lambda: 0.0)  # steering angle at the front wheel";
        # "steering angle at the front wheel (rad)";
        self.beta = self.var(lambda: math.atan((self.lr/(self.lf + self.lr)) * math.tan(self.deltaf())))
        self.der('x', lambda: self.v() * math.cos(self.psi() + self.beta()))
        self.der('y', lambda: self.v() * math.sin(self.psi() + self.beta()))
        self.der('psi', self.get_der_psi)
        self.der('v', lambda: self.a())
        self.save()

    def get_der_psi(self):
        return (self.v()/self.lr) * math.sin(self.beta())
