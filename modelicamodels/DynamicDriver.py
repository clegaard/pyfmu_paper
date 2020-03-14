from Model import Model


class DynamicDriver(Model):

    def __init__(self):
        super().__init__()
        self.width = self.parameter(0.335)
        self.amplitude = self.parameter(0.8)
        self.risingtime = self.parameter(5.0)
        self.starttime = self.parameter(5.0)
        self.steering = self.var(self.get_steering)
        self.save()

    def get_steering(self):
        if self.time() < self.starttime:
            return 0.0
        elif (self.time() - self.starttime) < self.risingtime:
            return self.amplitude * (self.time() - self.starttime) / self.risingtime
        elif (self.time() - self.starttime - self.risingtime) < self.width:
            return self.amplitude
        else:
            return max(0.0, self.amplitude - self.amplitude * (self.time() - self.starttime - self.risingtime - self.width) / self.risingtime)


