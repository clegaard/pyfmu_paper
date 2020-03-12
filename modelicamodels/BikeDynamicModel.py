import math
from scipy.integrate import odeint
import numpy as np


class BikeDynamicModel:

    def __init__(self):
        super().__init__()
        self.lf = 1.105  # distance from the the center of mass to the front (m)";
        self.lr = 1.738  # distance from the the center of mass to the rear (m)";
        self.m = 1292.2  # Vehicle's mass (kg)";
        self.Iz = 1.0  # Yaw inertial (kgm^2) (Not taken from the book)";
        self.Caf = 800.0  # Front Tire cornering stiffness";
        self.Car = 800.0  # Rear Tire cornering stiffness";
        self.x0 = 0.0  # initial longitudinal displacement";
        self.y0 = 0.0  # initial lateral displacement";
        self.X0 = 0.0  # initial position in x";
        self.Y0 = 0.0  # initial position in y";
        self.psi0 = 0.0  # initial yaw";
        self.dpsi0 = 0.0  # initial yaw rate";
        self.vx0 = 0.0  # initial velocity along x";
        self.vy0 = 0.0  # initial velocity along x";

        self.ts = []
        self.af = []  # Front Tire slip angle";
        self.ar = []  # Rear Tire slip angle";
        self.x = []  # longitudinal displacement in the body frame";
        self.X = []  # x coordinate in the reference frame";
        self.Y = []  # x coordinate in the reference frame";
        self.vx = []  # velocity along x";
        self.y = []  # lateral displacement in the body frame";
        self.vy = []  # velocity along y";
        self.psi = []  # Yaw";
        self.dpsi = []  # Yaw rate";
        self.Fcf = []  # lateral tire force at the front tire in the frame of the front tire";
        self.Fcr = []  # lateral tire force at the rear tire in the frame of the rear tire";
        # Inputs are functions that can be called
        self.a = None  # longitudinal acceleration";
        self.deltaf = None  # steering angle at the front wheel";

    def compute_initial_state(self):
        return [
            self.x0,
            self.y0,
            self.X0,
            self.Y0,
            self.vx0,
            self.vy0,
            self.psi0,
            self.dpsi0
        ]

    def get_af(self, deltaf, vy, dpsi, vx):
        return deltaf - (vy + self.lf * dpsi) / vx

    def get_ar(self, vy, dpsi, vx):
        return (vy - self.lr * dpsi) / vx

    def get_Fcf(self, af):
        return self.Caf * af

    def get_Fcr(self, ar):
        return self.Car * (-ar)

    def compute_algebraic_signals(self):
        npoints = len(self.x)

        self.af = np.zeros(npoints)
        self.ar = np.zeros(npoints)
        self.Fcf = np.zeros(npoints)
        self.Fcr = np.zeros(npoints)

        for i in range(npoints):
            self.af[i] = self.get_af(self.deltaf(self.ts[i]), self.vy[i], self.dpsi[i], self.vx[i])
            self.ar[i] = self.get_ar(self.vy[i], self.dpsi[i], self.vx[i])
            self.Fcf[i] = self.get_Fcf(self.af[i])
            self.Fcr[i] = self.get_Fcr(self.ar[i])

    def get_model(self):
        def derivatives(state, t):
            # Get state
            x = state[0]
            y = state[1]
            X = state[2]
            Y = state[3]
            vx = state[4]
            vy = state[5]
            psi = state[6]
            dpsi = state[7]

            # Get inputs
            a = self.a(t)
            deltaf = self.deltaf(t)

            # Algebraic formulas
            af = self.get_af(deltaf, vy, dpsi, vx)
            ar = self.get_ar(vy, dpsi, vx)
            Fcf = self.get_Fcf(af)
            Fcr = self.get_Fcr(ar)

            # Derivatives
            der_x = vx
            der_y = vy
            der_X = vx * math.cos(psi) - vy * math.sin(psi)
            der_Y = vx * math.sin(psi) + vy * math.cos(psi)
            # Longitudinal dynamics are the same as the bike model.
            der_vx = dpsi * vy + a
            der_vy = -dpsi * vx + (1 / self.m) * (Fcf * math.cos(deltaf) + Fcr)
            der_psi = dpsi
            der_dpsi = (2 / self.Iz) * (self.lf * Fcf - self.lr * Fcr)

            ders = [
                der_x,
                der_y,
                der_X,
                der_Y,
                # Longitudinal dynamics are the same as the bike model.
                der_vx,
                der_vy,
                der_psi,
                der_dpsi
            ]

            return ders

        return derivatives

    def simulate(self, stop_time, step):
        ts = np.arange(0, stop_time, step)

        initial = self.compute_initial_state()

        sol = odeint(self.get_model(), initial, ts)

        self.x = sol[:, 0]
        self.y = sol[:, 1]
        self.X = sol[:, 2]
        self.Y = sol[:, 3]
        self.vx = sol[:, 4]
        self.vy = sol[:, 5]
        self.psi = sol[:, 6]
        self.dpsi = sol[:, 7]
        self.ts = ts

        self.compute_algebraic_signals()




