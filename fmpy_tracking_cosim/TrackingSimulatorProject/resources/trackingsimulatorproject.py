import math

from oomodelling.Model import Model
from oomodelling.ModelSolver import StepRK45
from pyfmu.fmi2 import Fmi2Slave,Fmi2Causality, Fmi2Variability,Fmi2DataTypes,Fmi2Initial
from scipy.integrate import solve_ivp
from oomodelling.ModelSolver import ModelSolver
from oomodelling.TrackingSimulator import TrackingSimulator
import numpy as np
import matplotlib.pyplot as plt


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
                               start=0.0,
                               description="",
                               define_attribute=True,
                               value_reference=None)
        self.Y = 0.0
        self.register_variable("Y",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.input,
                               variability=Fmi2Variability.continuous,
                               initial=None,
                               start=0.0,
                               description="",
                               define_attribute=True,
                               value_reference=None)
        self.vx = 1.0
        self.register_variable("vx",
                               data_type=Fmi2DataTypes.real,
                               causality=Fmi2Causality.input,
                               variability=Fmi2Variability.continuous,
                               initial=None,
                               start=1.0,
                               description="",
                               define_attribute=True,
                               value_reference=None)

        # Internal model
        self.model = RobottiTrackingSimulator()

        self.model.dbike.deltaf = lambda: self.steering
        self.model.steering = lambda: self.steering
        self.model.to_track_X = lambda: self.X
        self.model.to_track_Y = lambda: self.Y
        self.model.to_track_vx = lambda: self.vx
        self.model.dbike.vx = lambda: self.vx

        # TODO: Move these to the parameters.
        self.model.tolerance = 0.01
        self.model.horizon = 2.0
        self.model.max_iterations = 20
        self.model.cooldown = 5.0
        self.model.nsamples = 20
        self.model.time_step = 0.1
        self.model.conv_xatol = 1e7
        self.model.conv_fatol = 1e-3

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

        self.start_time = 0.0

    def setup_experiment(self,
                         start_time: float,
                         tolerance: float = None,
                         stop_time: float = None):
        self.start_time = start_time
        self.model.set_time(start_time)

    def exit_initialization_mode(self):
        self.model.assert_initialized()
        x = self.model.state_vector()
        self.model.record_state(x, self.start_time)

    def do_step(self, current_time: float, step_size: float, no_set_fmu_state_prior: bool) -> bool:
        try:
            f = self.model.derivatives()
            x = self.model.state_vector()
            assert np.isclose(self.model.time(), current_time), "np.isclose(self.model.time(), current_time)"
            assert np.isclose(self.model.dbike.time(), current_time), "np.isclose(self.model.dbike.time(), current_time)"
            sol = solve_ivp(f, (current_time, current_time+step_size), x, method=StepRK45, max_step=step_size, model=self.model, t_eval=[current_time+step_size])
            assert sol.success
            assert np.isclose(self.model.time(), current_time + step_size), "np.isclose(self.model.time(), current_time + step_size)"
            assert np.isclose(self.model.dbike.time(), current_time + step_size), "np.isclose(self.model.dbike.time(), current_time + step_size)"

            self.tracking_X = self.model.dbike.X()
            self.tracking_Y = self.model.dbike.Y()
        except Exception as e:
            raise e

        return True

    def reset(self):
        raise NotImplementedError()

    def terminate(self):
        pass


class RobottiTrackingSimulator(TrackingSimulator):

    def __init__(self):
        super().__init__()

        self.steering = self.input(lambda: 0.0)
        self.to_track_X = self.input(lambda: 0.0)
        self.to_track_Y = self.input(lambda: 0.0)
        self.to_track_vx = self.input(lambda: 1.0)

        self.dbike = self.get_new_bike_model()

        self.match_signals(self.to_track_X, self.dbike.X)
        self.match_signals(self.to_track_Y, self.dbike.Y)

        self.X_idx = self.dbike.get_state_idx('X')
        self.Y_idx = self.dbike.get_state_idx('Y')

        self.save()

    def get_new_bike_model(self):
        b = BikeDynamicModelSpeedDriven()
        b.m = 1700.0
        b.lf = 0.8525000000000001
        b.lr = 0.6974999999999999
        b.Iz = 2135.7808333333332
        b.Caf = lambda: 20000.0
        b.Car = 20000.0
        return b

    def run_whatif_simulation(self, new_parameters, t0, tf, tracked_solutions, error_space, only_tracked_state=True):
        new_caf = new_parameters[0]
        assert new_caf > 0.0, "Negative parameter?"
        m = self.get_new_bike_model()
        # Set new parameter
        m.Caf = lambda: new_caf
        # Rewrite control input to mimic the past behavior.
        m.deltaf = lambda: self.steering(-(tf - m.time()))
        m.vx = lambda: self.to_track_vx(-(tf - m.time()))

        assert np.isclose(self.to_track_X(-(tf - t0)), tracked_solutions[0][0])
        assert np.isclose(self.to_track_Y(-(tf - t0)), tracked_solutions[1][0])
        # Initialize the state to the state at t0
        m.x = self.dbike.x(-(tf - t0))
        m.X = self.to_track_X(-(tf - t0))
        m.y = self.dbike.y(-(tf - t0))
        m.Y = self.to_track_Y(-(tf - t0))
        m.vx = self.dbike.vx(-(tf - t0))
        m.vy = self.dbike.vy(-(tf - t0))
        m.psi = self.dbike.psi(-(tf - t0))
        m.dpsi = self.dbike.dpsi(-(tf - t0))

        sol = ModelSolver().simulate(m, t0, tf, self.time_step, error_space)
        new_trajectories = sol.y
        if only_tracked_state:
            new_trajectories = np.array([
                sol.y[self.X_idx, :],
                sol.y[self.Y_idx, :]
            ])
            assert len(new_trajectories) == 2
            assert len(new_trajectories[0, :]) == len(sol.y[0, :])

        # _, (p1, p2, p3) = plt.subplots(1, 3)
        # p1.plot(m.signals['time'], m.signals['deltaf'], label='steering')
        # p1.plot(m.signals['time'], [self.steering(-(tf - t)) for t in m.signals['time']], label='steering')
        # p1.legend()
        # p2.plot(m.signals['X'], m.signals['Y'], label='~dX vs ~dY')
        # p2.legend()
        # p3.plot(m.signals['time'], m.signals['Caf'], label='approx_Caf')
        # p3.legend()
        # plt.show()

        return new_trajectories

    def update_tracking_model(self, new_present_state, new_parameter):
        self.dbike.record_state(new_present_state, self.time(), override=True)
        self.dbike.Caf = lambda: new_parameter[0]
        assert np.isclose(new_present_state[self.X_idx], self.dbike.X())
        assert np.isclose(new_present_state[self.Y_idx], self.dbike.Y())

    def get_parameter_guess(self):
        return np.array([self.dbike.Caf()])


class BikeDynamicModelSpeedDriven(Model):
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
        self.y = self.state(0.0)  # lateral displacement in the body frame";
        self.vy = self.state(0.0)  # velocity along y";
        self.psi = self.state(0.0)  # Yaw";
        self.dpsi = self.state(0.0)  # Yaw rate";
        self.vx = self.input(lambda: 1.0)  # velocity along x;
        self.deltaf = self.input(lambda: 0.0) # steering angle at the front wheel";
        self.af = self.var(self.get_af)  # Front Tire slip angle";
        self.ar = self.var(lambda: (self.vy() - self.lr*self.dpsi())/self.vx())  # Rear Tire slip angle";
        self.Fcf = self.var(lambda: self.Caf()*self.af())  # lateral tire force at the front tire in the frame of the front tire";
        self.Fcr = self.var(lambda: self.Car*(-self.ar()))  # lateral tire force at the rear tire in the frame of the rear tire";

        self.der('x', lambda: self.vx())
        self.der('y', lambda: self.vy())
        self.der('psi', lambda: self.dpsi())
        self.der('vy', lambda: -self.dpsi()*self.vx() + (1/self.m)*(self.Fcf() * math.cos(self.deltaf()) + self.Fcr()))
        self.der('dpsi', lambda: (2/self.Iz)*(self.lf*self.Fcf() - self.lr*self.Fcr()))
        self.der('X', lambda: self.vx()*math.cos(self.psi()) - self.vy()*math.sin(self.psi()))
        self.der('Y', lambda: self.vx()*math.sin(self.psi()) + self.vy()*math.cos(self.psi()))

        self.save()

    def get_af(self):
        vx = self.vx()
        delta_f = self.deltaf()
        time = self.time()
        res = delta_f - (self.vy() + self.lf*self.dpsi())/vx
        return res

if __name__ == '__main__':
    slave = TrackingSimulatorProject()
