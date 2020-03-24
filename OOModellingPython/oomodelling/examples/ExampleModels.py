import math

from oomodelling.Model import Model


class MassDamper(Model):
    def __init__(self):
        super().__init__()

        self.x = self.state(0.0)
        self.v = self.state(1.0)
        self.d = self.parameter(1.0)

        self.friction = self.var(lambda: self.d * self.v())

        self.F = self.input(lambda: 0.0)

        self.der('x', lambda: self.v())
        self.der('v', lambda: self.F() - self.friction())

        self.save()


class Spring(Model):
    def __init__(self):
        super().__init__()

        self.x = self.input(lambda: 0.0)
        self.k = self.parameter(1.0)
        self.F = self.var(lambda: - self.k * self.x())
        self.save()


class MassSpringDamper(Model):
    def __init__(self):
        super().__init__()

        self.md = MassDamper()
        self.s = Spring()

        self.s.x = self.md.x
        self.md.F = self.s.F

        self.save()


class MassSpringDamperFlat(Model):
    def __init__(self):
        super().__init__()
        self.x = self.state(0.0)
        self.v = self.state(1.0)

        self.F = self.input(lambda: 0.0)

        self.k = self.parameter(1.0)
        self.d = self.parameter(1.0)
        self.spring = self.var(lambda: self.k * self.x())
        self.damper = self.var(lambda: self.d * self.v())
        self.der('x', lambda: self.v())
        self.der('v', lambda: self.F() - self.damper() - self.spring())
        self.save()


class MSDAutonomous(Model):
    def __init__(self):
        super().__init__()
        self.x = self.state(0.0)
        self.v = self.state(1.0)

        self.k = self.parameter(1.0)
        self.d = self.parameter(1.0)
        self.der('x', lambda: self.v())
        self.der('v', lambda: - self.d * self.v() - self.k * self.x())

        self.save()


class TimeDepInput(Model):
    def __init__(self):
        super().__init__()
        self.F = self.var(lambda: 0.0 if self.time() < 4.0 else 4.0)
        self.save()


class DelayExample(Model):
    def __init__(self):
        super().__init__()
        self.u = self.input(lambda: 0.0)
        self.d = self.var(lambda: self.u(-1.0))
        self.save()


class DelayRewireInput(Model):
    def __init__(self):
        super().__init__()
        self.d = DelayExample()
        self.d.u = lambda: math.sin(self.time())
        self.save()


class DelayExampleScenario(Model):
    def __init__(self):
        super().__init__()
        self.u = TimeDepInput()
        self.d = DelayExample()

        self.d.u = self.u.F

        self.save()


class MSDTimeDep(Model):
    def __init__(self):
        super().__init__()

        self.msd = MassSpringDamperFlat()
        self.u = TimeDepInput()

        self.msd.F = self.u.F

        self.save()


class TwoMSDComparison(Model):
    def __init__(self):
        super().__init__()

        self.m1 = MassSpringDamperFlat()
        self.m2 = MassSpringDamper()
        self.save()