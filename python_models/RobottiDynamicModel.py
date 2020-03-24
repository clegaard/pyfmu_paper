import math

from oomodelling.Model import Model


class RobottiDynamicModel(Model):
    def __init__(self):
        super().__init__()
        self.T = self.parameter(3.56)  # Track width
        self.L = self.parameter(1.55)  # Wheel base
        self.front_to_rear_ratio = self.parameter(0.55)
        self.lr_ratio = self.parameter(0.5)
        self.Caf = self.input(lambda: 20000)  # Front Tire cornering stiffness";
        self.mu = self.parameter(0.6)  # surface friction
        self.l_ext = self.parameter(1.5)
        self.h = self.parameter(1.0)
        self.m = self.parameter(1700.0)  # Vehicle's mass (kg)";
        self.m_robot_in = self.parameter(self.m)  # Robot's actual mass (simplified)
        self.m_robot_l = self.parameter(self.m_robot_in*(1.0-self.lr_ratio))  # Mass at the left
        self.m_robot_r = self.parameter(self.m_robot_in*(self.lr_ratio))  # Mass on the right
        self.Iz = self.parameter(self.m_total/12.0*(self.T**2+self.L**2))  # Yaw inertial
        self.lf = self.parameter(self.L*self.front_to_rear_ratio)  # distance from the the center of mass to the front (m)";
        self.lr = self.parameter(self.L*(1.0-self.front_to_rear_ratio))  # distance from the the center of mass to the rear (m)";
        self.Car = self.parameter(20000)  # Rear Tire cornering stiffness";

        self.x = self.state(0.0)  # longitudinal displacement in the body frame";
        self.X = self.state(0.0)  # x coordinate in the reference frame";
        self.Y = self.state(0.0)  # x coordinate in the reference frame";
        self.vx = self.state(1.0)  # velocity along x";
        self.y = self.state(0.0)  # lateral displacement in the body frame";
        self.vy = self.state(0.0)  # velocity along y";
        self.psi = self.state(0.0)  # Yaw";
        self.dpsi = self.state(0.0)  # Yaw rate";
        self.a = self.input(lambda: 0.0)  # longitudinal acceleration";
        self.deltaFl = self.input(lambda: 0.0)  # steering angle at the front left wheel";
        self.deltaFr = self.input(lambda: 0.0)  # steering angle at the front right wheel";
        self.deltaRl = self.input(lambda: 0.0)  # steering angle at the rear left wheel";
        self.deltaRr = self.input(lambda: 0.0)  # steering angle at the rear right wheel";

        self.ur = self.var(lambda: self.vx())
        self.alphaFl = self.var(lambda: math.atan((self.vy() + self.lf*self.dpsi()) / (self.vx() + ((self.T/2) * self.dpsi())))-self.deltaFl())
        self.alphaFr = self.var(lambda: math.atan((self.vy() + self.lf*self.dpsi()) / (self.vx() - ((self.T/2) * self.dpsi())))-self.deltaFr())
        self.alphaRl = self.var(lambda: math.atan((self.vy() - self.lf*self.dpsi()) / (self.vx() + ((self.T/2) * self.dpsi())))-self.deltaRl())
        self.alphaRr = self.var(lambda: math.atan((self.vy() - self.lf*self.dpsi()) / (self.vx() - ((self.T/2) * self.dpsi())))-self.deltaRr())





        self.af = self.var(lambda: self.deltaf() - ( self.vy() + self.lf*self.dpsi())/self.vx())  # Front Tire slip angle";
        self.ar = self.var(lambda: (self.vy() - self.lr*self.dpsi())/self.vx())  # Rear Tire slip angle";
        self.Fcf = self.var(lambda: self.Caf()*self.af())  # lateral tire force at the front tire in the frame of the front tire";
        self.Fcr = self.var(lambda: self.Car*(-self.ar()))  # lateral tire force at the rear tire in the frame of the rear tire";

        self.der('x', lambda: self.vx())
        self.der('y', lambda: self.vy())
        self.der('psi', lambda: self.dpsi())
        self.der('vx', lambda: self.dpsi()*self.vy() + self.a())
        self.der('vy', lambda: -self.dpsi()*self.vx() + (1/self.m)*(self.Fcf() * math.cos(self.deltaf()) + self.Fcr()))
        self.der('dpsi', lambda: (1/self.Iz)*(self.lf*self.Fcf() - self.lr*self.Fcr()))
        self.der('X', lambda: self.vx()*math.cos(self.psi()) - self.vy()*math.sin(self.psi()))
        self.der('Y', lambda: self.vx()*math.sin(self.psi()) + self.vy()*math.cos(self.psi()))

        self.save()





