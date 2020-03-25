import math

from oomodelling.Model import Model


class RobottiDynamicModel(Model):
    def __init__(self):
        super().__init__()
        self.T = self.parameter(3.56)  # Track width
        self.L = self.parameter(1.55)  # Wheel base
        self.front_to_rear_ratio = self.parameter(0.55)
        self.lr_ratio = self.parameter(0.5)
        self.Car = self.parameter(20000)  # Rear Tire cornering stiffness";
        self.mu = self.parameter(0.6)  # surface friction
        self.l_ext = self.parameter(1.5)
        self.h = self.parameter(1.0)
        self.m = self.parameter(1700.0)  # Vehicle's mass (kg)";
        self.m_robot_in = self.parameter(self.m)  # Robot's actual mass (simplified)
        self.m_robot_l = self.parameter(self.m_robot_in*(1.0-self.lr_ratio))  # Mass at the left
        self.m_robot_r = self.parameter(self.m_robot_in*(self.lr_ratio))  # Mass on the right
        self.m_total = self.parameter(self.m_robot_in)
        self.Iz = self.parameter(self.m_total/12.0*(self.T**2+self.L**2))  # Yaw inertial
        self.lf = self.parameter(self.L*self.front_to_rear_ratio)  # distance from the the center of mass to the front (m)";
        self.lr = self.parameter(self.L*(1.0-self.front_to_rear_ratio))  # distance from the the center of mass to the rear (m)";
        self.g = self.parameter(9.81)  # Gravity
        self.M_ext = self.parameter(0.0)
        self.m_ext_l = self.parameter(0.0)
        self.m_ext_r = self.parameter(0.0)
        self.Nlrl = self.parameter((self.lf*self.m_robot_l*self.g - self.M_ext) / self.L)  # Normal load rear left
        self.Nlrr = self.parameter(self.m_robot_r*self.g*(self.lf/self.L))  # Normal load on the rear right
        self.Nlfr = self.parameter(self.m_robot_r*self.g*(self.lr/self.L)+self.m_ext_r*self.g)  # Normal load on the front right
        self.Nlfl = self.parameter(1/self.L*(self.m_robot_l*self.g*self.lr+self.m_ext_l*self.g*(self.L-self.l_ext))) # ...

        # Steering parameters
        self.wheel_radius = self.parameter(0.29)
        self.Cs = self.parameter(-0.055)
        self.SC = self.parameter(0.95)

        self.x = self.state(0.0)  # longitudinal displacement in the body frame";
        self.X = self.state(0.0)  # x coordinate in the reference frame";
        self.Y = self.state(0.0)  # x coordinate in the reference frame";
        self.y = self.state(0.0)  # lateral displacement in the body frame";
        self.vy = self.state(0.0)  # velocity along y";
        self.psi = self.state(0.0)  # Yaw";
        self.dpsi = self.state(0.0)  # Yaw rate";
        self.Caf = self.input(lambda: 20000.0)  # Front Tire cornering stiffness";

        self.deltaFl = self.input(lambda: 0.0)  # steering angle at the front left wheel";
        self.deltaFr = self.input(lambda: 0.0)  # steering angle at the front right wheel";
        self.deltaRl = self.input(lambda: 0.0)  # steering angle at the rear left wheel";
        self.deltaRr = self.input(lambda: 0.0)  # steering angle at the rear right wheel";
        self.vel_left = self.input(lambda: 1.0)  # speed of the left wheel
        self.vel_right = self.input(lambda: 1.0)  # speed of the right wheel

        self.omega = self.var(lambda: (self.vel_left()-self.vel_right())*self.wheel_radius * (self.T/2) / self.wheel_radius*self.Cs)
        self.vx = self.var(lambda: (self.vel_left()+self.vel_right())*self.wheel_radius/2 * self.SC)

        # Slip angle for each wheel
        self.alphaFl = self.var(lambda: math.atan((self.vy() + self.lf*self.dpsi()) / (self.vx() + ((self.T/2) * self.dpsi())))-self.deltaFl())
        self.alphaFr = self.var(lambda: math.atan((self.vy() + self.lf*self.dpsi()) / (self.vx() - ((self.T/2) * self.dpsi())))-self.deltaFr())
        self.alphaRl = self.var(lambda: math.atan((self.vy() - self.lf*self.dpsi()) / (self.vx() + ((self.T/2) * self.dpsi())))-self.deltaRl())
        self.alphaRr = self.var(lambda: math.atan((self.vy() - self.lf*self.dpsi()) / (self.vx() - ((self.T/2) * self.dpsi())))-self.deltaRr())

        def sign(x):
            if x > 0:
                return 1.0
            elif x < 0:
                return -1.0
            else:
                return 0.0

        # Front left tyre non linear model
        self.useNLFLT = self.var(lambda: abs(-self.Caf() * math.tan(self.alphaFl())) - self.mu * self.Nlfl / 2)
        self.FyflT = self.var(lambda: -self.mu*self.Nlfr*sign(self.alphaFl())*(1-(self.mu*self.Nlfr) / (4*self.Caf()*abs(math.tan(self.alphaFl())))) if self.useNLFLT() > 0 else -self.Caf()*math.tan(self.alphaFl()))

        # Front right tyre non linear model
        self.useNLFRT = self.var(lambda: abs(-self.Caf()*math.tan(self.alphaFr())) - self.mu*self.Nlfr/2)
        self.FyfrT = self.var(lambda: -self.mu*self.Nlfr*sign(self.alphaFr())*(1-(self.mu*self.Nlfr) / (4*self.Caf()*abs(math.tan(self.alphaFr())))) if self.useNLFRT() > 0 else -self.Caf()*math.tan(self.alphaFr()))

        # Rear left tyre non linear model
        self.useNLRLT = self.var(lambda: abs(-self.Caf()*math.tan(self.alphaRl())) - self.mu*self.Nlrl/2)
        self.FyrlT = self.var(lambda: -self.mu*self.Nlrl*sign(self.alphaRl())*(1-(self.mu*self.Nlrl) / (4*self.Car*abs(math.tan(self.alphaRl())))) if self.useNLRLT() > 0 else -self.Car*math.tan(self.alphaRl()))

        # Rear right tyre non linear model
        self.useNLRRT = self.var(lambda: abs(-self.Car * math.tan(self.alphaRr())) - self.mu * self.Nlrr / 2)
        self.FyrrT = self.var(lambda: -self.mu*self.Nlrr*sign(self.alphaRr())*(1-(self.mu*self.Nlrr) / (4*self.Car*abs(math.tan(self.alphaRr())))) if self.useNLRRT() > 0 else -self.Car * math.tan(self.alphaRr()))

        # Lateral forces on each tyre.
        # self.Fyfl = self.var(lambda: self.FyflT() * math.cos(self.deltaFl()) + 0 * math.sin(self.deltaFl()))
        # self.Fyfr = self.var(lambda: self.FyfrT() * math.cos(self.deltaFr()) + 0 * math.sin(self.deltaFr()))
        # self.Fyrl = self.var(lambda: self.FyrlT() * math.cos(self.deltaRl()) + 0 * math.sin(self.deltaRl()))
        # self.Fyrr = self.var(lambda: self.FyrrT() * math.cos(self.deltaRr()) + 0 * math.sin(self.deltaRr()))

        self.Fyfl = self.var(lambda: self.FyflT())
        self.Fyfr = self.var(lambda: self.FyfrT())
        self.Fyrl = self.var(lambda: self.FyrlT())
        self.Fyrr = self.var(lambda: self.FyrrT())

        # Sum of the forces on front and read tyre.
        self.Fcf = self.var(lambda: self.Fyfl() + self.Fyfr())
        self.Fcr = self.var(lambda: self.Fyrl() + self.Fyrr())

        # Dynamics
        self.der('x', lambda: self.vx())
        self.der('y', lambda: self.vy())
        self.der('psi', lambda: self.dpsi())
        self.der('vy', lambda: (1/self.m)*(self.Fcf() + self.Fcr()) - self.dpsi()*self.vx())
        self.der('dpsi', lambda: (1/self.Iz)*(self.lf*self.Fcf() - self.lr*self.Fcr()) + self.omega())
        self.der('X', lambda: self.vx()*math.cos(self.psi()) - self.vy()*math.sin(self.psi()))
        self.der('Y', lambda: self.vx()*math.sin(self.psi()) + self.vy()*math.cos(self.psi()))

        self.save()


