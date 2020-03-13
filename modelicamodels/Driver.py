from Model import Model


class Driver(Model):

    def __init__(self):
        super().__init__()
        self.parameter('width', 0.335)
        self.parameter('amplitude', 0.8)
        self.parameter('risingtime', 5.0)
        self.parameter('starttime', 5.0)
        self.var('steering', self.get_steering)

    def get_steering(self):
        if self.time < self.starttime:
            return 0.0
        elif (self.time - self.starttime) < self.risingtime:
            return self.amplitude * (self.time - self.starttime) / self.risingtime
        elif (self.time - self.starttime - self.risingtime) < self.width:
            return self.amplitude
        else:
            return max(0.0, self.amplitude - self.amplitude * (self.time - self.starttime - self.risingtime - self.width) / self.risingtime)


