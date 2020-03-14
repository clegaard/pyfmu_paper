from Model import Model


class Solver:
    def __init__(self):
        super().__init__()

    """
    Computes the behavior of the model.
    Uses a forward euler solver because we need to progagate intermediate signals to the different components.
    Operators such as delay need this information. 
    Using an off the shelf solver for this makes it impossible to get the real intermediate states, 
    due to the many intermediate model evaluations.
    """
    def simulate(self, model: Model, t0, tf, h):
        pass

