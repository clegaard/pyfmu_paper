import unittest
from fmpy import *

from cosimlibrary.loader import FMULoader

class MyTestCase(unittest.TestCase):

    def test_dump(self):
        dump('TrackingSimulator.fmu')

    def test_load(self):
        def logger(msg):
            print(msg)

        tracking = FMULoader.load("TrackingSimulator.fmu", "tracking", logger)
        print(tracking.vars)

        robotti = FMULoader.load("robotti_global.fmu", "robotti", logger)
        print(robotti.vars)

        steering = FMULoader.load("Steering_input.fmu", "steering", logger)
        print(steering.vars)


if __name__ == '__main__':
    unittest.main()
