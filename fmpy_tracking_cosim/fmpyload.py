import unittest
from fmpy import *

class MyTestCase(unittest.TestCase):
    def test_something(self):
        dump('TrackingSimulator.fmu')


if __name__ == '__main__':
    unittest.main()
