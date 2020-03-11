from pyfmu.fmi2slave import Fmi2Slave
from pyfmu.fmi2types import Fmi2Causality, Fmi2Variability, Fmi2DataTypes, Fmi2Initial


from scipy.integrate import odeint


class bicycle_model(Fmi2Slave):

    def __init__(self):

        author = ""
        modelName = "bicycle_model"
        description = ""

        super().__init__(
            modelName=modelName,
            author=author,
            description=description)

        # model
        self.register_variable("a", Fmi2DataTypes.real,
                               Fmi2Causality.input, start=0)
        self.register_variable("df", Fmi2DataTypes.real,
                               Fmi2Causality.input, start=0)

        self.register_variable("x", Fmi2DataTypes.real, Fmi2Causality.output)
        self.register_variable("y", Fmi2DataTypes.real, Fmi2Causality.output)
        self.register_variable("psi", Fmi2DataTypes.real, Fmi2Causality.output)
        self.register_variable("v", Fmi2DataTypes.real, Fmi2Causality.output)
        self.register_variable(
            "beta", Fmi2DataTypes.real, Fmi2Causality.output)

        self.register_variable("x0", Fmi2DataTypes.real,
                               Fmi2Causality.parameter, Fmi2Variability.fixed, start=0)
        self.register_variable("y0", Fmi2DataTypes.real,
                               Fmi2Causality.parameter, Fmi2Variability.fixed, start=0)

        self.register_variable("psi0", Fmi2DataTypes.real,
                               Fmi2Causality.parameter, Fmi2Variability.fixed, start=0)

        self.register_variable("v0", Fmi2DataTypes.real,
                               Fmi2Causality.parameter, Fmi2Variability.fixed, start=0)

        self.register_variable("beta0", Fmi2DataTypes.real,
                               Fmi2Causality.parameter, Fmi2Variability.fixed, start=0)

        # reference

        self.register_variable("x_r", Fmi2DataTypes.real,
                               Fmi2Causality.input, start=0)
        self.register_variable("y_r", Fmi2DataTypes.real,
                               Fmi2Causality.input, start=0)
        self.register_variable(
            "psi_r", Fmi2DataTypes.real, Fmi2Causality.input, start=0)
        self.register_variable("v_r", Fmi2DataTypes.real,
                               Fmi2Causality.input, start=0)
        self.register_variable(
            "beta_r", Fmi2DataTypes.real, Fmi2Causality.input, start=0)

    def setup_experiment(self, start_time: float):
        pass

    def enter_initialization_mode(self):
        pass

    def exit_initialization_mode(self):
        self.x = self.x0
        self.y = self.y0
        self.psi = self.psi0
        self.v = self.v0
        self.beta = self.beta0

    def do_step(self, current_time: float, step_size: float) -> bool:
        return True

    def reset(self):
        pass

    def terminate(self):
        pass


# validation
if __name__ == "__main__":
    model = bicycle_model()
