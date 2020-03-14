import math

from Model import Model


class BikeDynamicModel(Model):
    def __init__(self):
        super().__init__()
        self.parameter('lf', 1.105) # distance from the the center of mass to the front (m)";
        self.parameter('lr', 1.738) # distance from the the center of mass to the rear (m)";
        self.parameter('m', 1292.2) # Vehicle's mass (kg)";
        self.parameter('Iz', 1) # Yaw inertial (kgm^2) (Not taken from the book)";
        self.parameter('Caf', 800) # Front Tire cornering stiffness";
        self.parameter('Car', 800) # Rear Tire cornering stiffness";
        self.state('x', 0.0) # longitudinal displacement in the body frame";
        self.state('X', 0.0) # x coordinate in the reference frame";
        self.state('Y', 0.0) # x coordinate in the reference frame";
        self.state('vx', 1.0) # velocity along x";
        self.state('y', 0.0) # lateral displacement in the body frame";
        self.state('vy', 0.0) # velocity along y";
        self.state('psi', 0.0) # Yaw";
        self.state('dpsi', 0.0) # Yaw rate";
        self.input('a') # longitudinal acceleration";
        self.input('deltaf') # steering angle at the front wheel";
        self.var('af', lambda: self.deltaf() - ( self.vy + self.lf*self.dpsi)/self.vx) # Front Tire slip angle";
        self.var('ar', lambda: (self.vy - self.lr*self.dpsi)/self.vx) # Rear Tire slip angle";
        self.var('Fcf', lambda: self.Caf*self.af()) # lateral tire force at the front tire in the frame of the front tire";
        self.var('Fcr', lambda: self.Car*(-self.ar())) # lateral tire force at the rear tire in the frame of the rear tire";

        self.der('x', lambda: self.vx)
        self.der('y', lambda: self.vy)
        self.der('psi', lambda: self.dpsi)
        self.der('vx', lambda: self.dpsi*self.vy + self.a())
        self.der('vy', lambda: -self.dpsi*self.vx + (1/self.m)*(self.Fcf() * math.cos(self.deltaf()) + self.Fcr()))
        self.der('dpsi', lambda: (2/self.Iz)*(self.lf*self.Fcf() - self.lr*self.Fcr()))
        self.der('X', lambda: self.vx*math.cos(self.psi) - self.vy*math.sin(self.psi))
        self.der('Y', lambda: self.vx*math.sin(self.psi) + self.vy*math.cos(self.psi))




