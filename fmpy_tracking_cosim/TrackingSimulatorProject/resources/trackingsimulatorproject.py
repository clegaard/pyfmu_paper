import math

from oomodelling.Model import Model
from oomodelling.ModelSolver import StepRK45
from pyfmu.fmi2 import Fmi2Slave,Fmi2Causality, Fmi2Variability,Fmi2DataTypes,Fmi2Initial
from scipy.integrate import solve_ivp
from oomodelling.ModelSolver import ModelSolver
from oomodelling.TrackingSimulator import TrackingSimulator
import numpy as np


class TrackingSimulatorProject(Fmi2Slave):

    def __init__(self):
        
        author = ""
        modelName = "TrackingSimulatorProject"
        description = ""    
        
        super().__init__(
            modelName=modelName,
            author=author,
            description=description)

        """
        Inputs, outputs and parameters may be defined using the 'register_variable' function:

        self.register_variable("my_input", data_type=Fmi2DataTypes.real, causality = Fmi2Causality.input, start=0)
        self.register_variable("my_output", data_type=Fmi2DataTypes.real, causality = Fmi2Causality.output)
        self.register_variable("my_parameter", data_type=Fmi2DataTypes.real, causality = Fmi2Causality.parameter, start=0)
        """

        # Inputs
        self.steering = 0.0
        self.register_variable("steering",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.input,
                               variability=Fmi2Variability.continuous,
                               initial=None,
                               start=None,
                               description="",
                               define_attribute=True,
                               value_reference=None)
        self.X = 0.0
        self.register_variable("X",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.input,
                               variability=Fmi2Variability.continuous,
                               initial=None,
                               start=None,
                               description="",
                               define_attribute=True,
                               value_reference=None)
        self.Y = 0.0
        self.register_variable("Y",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.input,
                               variability=Fmi2Variability.continuous,
                               initial=None,
                               start=None,
                               description="",
                               define_attribute=True,
                               value_reference=None)

        # Internal model
        self.model = BikeTracking()

        self.model.to_track_steering = lambda: self.steering
        self.model.to_track_X = lambda: self.X
        self.model.to_track_Y = lambda: self.Y

        # Outputs
        self.tracking_X = 0.0
        self.register_variable("tracking_X",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.output,
                               variability=Fmi2Variability.continuous,
                               initial=Fmi2Initial.exact,
                               start=0.0,
                               description="",
                               define_attribute=True,
                               value_reference=None)
        self.tracking_Y = 0.0
        self.register_variable("tracking_Y",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.output,
                               variability=Fmi2Variability.continuous,
                               initial=Fmi2Initial.exact,
                               start=0.0,
                               description="",
                               define_attribute=True,
                               value_reference=None)

        self.solver = None
        self.start_time = 0.0

    def setup_experiment(self, start_time: float):
        self.start_time = start_time
        self.model.set_time(start_time)

    def enter_initialization_mode(self):
        pass

    def exit_initialization_mode(self):
        self.model.assert_initialized()
        x = self.model.state_vector()
        self.model.record_state(x, self.start_time)

    def do_step(self, current_time: float, step_size: float, no_set_fmu_state_prior: bool) -> bool:
        f = self.model.derivatives()
        x = self.model.state_vector()
        sol = solve_ivp(f, (current_time, current_time+step_size), x, method=StepRK45, max_step=step_size, model=self.model, t_eval=None)
        assert sol.success

        self.tracking_X = self.model.tracking.X
        self.tracking_Y = self.model.tracking.Y

        return True

    def reset(self):
        NotImplemented

    def terminate(self):
        pass


class BikeTracking(TrackingSimulator):
    def __init__(self):
        super().__init__()

        self.tracking = BikeDynamicModel()

        self.to_track_steering = self.input(lambda: 0.0)
        self.to_track_X = self.input(lambda: 0.0)
        self.to_track_Y = self.input(lambda: 0.0)

        self.tracking.deltaf = self.to_track_steering

        self.match_signals(self.to_track_X, self.tracking.X)
        self.match_signals(self.to_track_Y, self.tracking.Y)

        self.X_idx = self.tracking.get_state_idx('X')
        self.Y_idx = self.tracking.get_state_idx('Y')

        self.save()

    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        new_caf = new_parameters[0]
        m = BikeDynamicModel()
        m.Caf = lambda: new_caf
        # Rewrite control input to mimic the past behavior.
        m.deltaf = lambda: self.to_track_steering(-(tf - m.time()))
        assert np.isclose(self.to_track_X(-(tf - t0)), tracked_solutions[0][0])
        assert np.isclose(self.to_track_Y(-(tf - t0)), tracked_solutions[1][0])
        m.x = self.tracking.x(-(tf - t0))
        m.X = self.tracking.X(-(tf - t0))
        m.y = self.tracking.y(-(tf - t0))
        m.Y = self.tracking.Y(-(tf - t0))
        m.vx = self.tracking.vx(-(tf - t0))
        m.vy = self.tracking.vy(-(tf - t0))
        m.psi = self.tracking.psi(-(tf - t0))
        m.dpsi = self.tracking.dpsi(-(tf - t0))

        sol = ModelSolver().simulate(m, t0, tf, self.time_step, error_space)
        new_trajectories = sol.y
        if only_tracked_state:
            new_trajectories = np.array([
                sol.y[self.X_idx, :],
                sol.y[self.Y_idx, :]
            ])
            assert len(new_trajectories) == 2
            assert len(new_trajectories[0, :]) == len(sol.y[0, :])

        return new_trajectories

    def update_tracking_model(self, new_present_state, new_parameter):
        self.tracking.record_state(new_present_state, self.time(), override=True)
        self.tracking.Caf = lambda: new_parameter[0]
        assert np.isclose(new_present_state[self.X_idx], self.tracking.X())
        assert np.isclose(new_present_state[self.Y_idx], self.tracking.Y())

    def get_parameter_guess(self):
        return np.array([self.tracking.Caf()])


class BikeDynamicModel(Model):
    def __init__(self):
        super().__init__()
        self.lf = self.parameter(1.105)  # distance from the the center of mass to the front (m)";
        self.lr = self.parameter(1.738)  # distance from the the center of mass to the rear (m)";
        self.m = self.parameter(1292.2)  # Vehicle's mass (kg)";
        self.Iz = self.parameter(1)  # Yaw inertial (kgm^2) (Not taken from the book)";
        self.Caf = self.input(lambda: 800)  # Front Tire cornering stiffness";
        self.Car = self.parameter(800)  # Rear Tire cornering stiffness";
        self.x = self.state(0.0)  # longitudinal displacement in the body frame";
        self.X = self.state(0.0)  # x coordinate in the reference frame";
        self.Y = self.state(0.0)  # x coordinate in the reference frame";
        self.vx = self.state(1.0)  # velocity along x";
        self.y = self.state(0.0)  # lateral displacement in the body frame";
        self.vy = self.state(0.0)  # velocity along y";
        self.psi = self.state(0.0)  # Yaw";
        self.dpsi = self.state(0.0)  # Yaw rate";
        self.a = self.input(lambda: 0.0)  # longitudinal acceleration";
        self.deltaf = self.input(lambda: 0.0) # steering angle at the front wheel";
        self.af = self.var(lambda: self.deltaf() - ( self.vy() + self.lf*self.dpsi())/self.vx())  # Front Tire slip angle";
        self.ar = self.var(lambda: (self.vy() - self.lr*self.dpsi())/self.vx())  # Rear Tire slip angle";
        self.Fcf = self.var(lambda: self.Caf()*self.af())  # lateral tire force at the front tire in the frame of the front tire";
        self.Fcr = self.var(lambda: self.Car*(-self.ar()))  # lateral tire force at the rear tire in the frame of the rear tire";

        self.der('x', lambda: self.vx())
        self.der('y', lambda: self.vy())
        self.der('psi', lambda: self.dpsi())
        self.der('vx', lambda: self.dpsi()*self.vy() + self.a())
        self.der('vy', lambda: -self.dpsi()*self.vx() + (1/self.m)*(self.Fcf() * math.cos(self.deltaf()) + self.Fcr()))
        self.der('dpsi', lambda: (2/self.Iz)*(self.lf*self.Fcf() - self.lr*self.Fcr()))
        self.der('X', lambda: self.vx()*math.cos(self.psi()) - self.vy()*math.sin(self.psi()))
        self.der('Y', lambda: self.vx()*math.sin(self.psi()) + self.vy()*math.cos(self.psi()))

        self.save()


if __name__ == '__main__':
    slave = TrackingSimulatorProject()
