import unittest
import matplotlib.pyplot as plt

from BikeDynamicModel import BikeDynamicModel


class TesDynamicBikeModel(unittest.TestCase):
    def test_basic_simulation(self):
        bike = BikeDynamicModel()

        bike.a = lambda t: 0.0
        bike.deltaf = lambda t: 0.8 * t / 5.0 if t < 5.0 else 0.0

        bike.vx0 = 1.0

        bike.simulate(10, 0.01)

        plt.plot(bike.ts, [bike.deltaf(t) for t in bike.ts], label='driver')
        plt.plot(bike.ts, bike.ar, label='ar')
        plt.plot(bike.ts, bike.X, label='X')

        plt.show()
