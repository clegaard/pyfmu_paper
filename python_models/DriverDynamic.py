from oomodelling.Model import Model


class DriverDynamic(Model):

    def __init__(self):
        super().__init__()
        self.width = self.parameter(0.335)
        self.amplitude = self.parameter(0.8)
        self.risingtime = self.parameter(5.0)
        self.starttime = self.parameter(5.0)
        self.steering = self.var(self.get_steering)
        self.last_period_endtime = 0
        self.nperiods = self.parameter(2)
        self.nperiods_counter = 0
        self.save()

    def get_steering(self):
        if self.nperiods_counter >= self.nperiods:
            return 0.0

        t = self.time() - self.last_period_endtime
        if t < self.starttime:
            return 0.0
        elif (t - self.starttime) < self.risingtime:
            # rising
            return self.amplitude * (t - self.starttime) / self.risingtime
        elif (t - self.starttime - self.risingtime) < self.width:
            # plateau
            return self.amplitude
        else:
            # falling
            res = self.amplitude - self.amplitude * (t - self.starttime - self.risingtime - self.width) / self.risingtime
            if res <= 0.0:
                # New period ended
                self.last_period_endtime = self.time()
                self.nperiods_counter += 1
            return max(0.0, res)


