from Model import Model


class MassDamper(Model):
    def __init__(self):
        super().__init__()
        self.state('x', 0.0)
        self.state('vx', 1.0)

        self.parameter('d', 1.0)
        self.var('friction', lambda: self.d * self.vx())
        self.input('F')
        self.der('x', lambda: self.vx())
        self.der('vx', lambda: self.F() - self.friction())


class Spring(Model):
    def __init__(self):
        super().__init__()

        self.input('x')
        self.parameter('k', 1.0)
        self.var('F', lambda: - self.k * self.x())


class MassSpringDamper(Model):
    def __init__(self):
        super().__init__()

        self.model('md', MassDamper())
        self.model('s', Spring())

        self.connect(self.s, 'x', self.md, 'x')
        self.connect(self.md, 'F', self.s, 'F')


class MassSpringDamperFlat(Model):
    def __init__(self):
        super().__init__()
        self.state('x', 0.0)
        self.state('v', 1.0)

        self.input('F')

        self.parameter('k', 1.0)
        self.parameter('d', 1.0)
        self.var('spring', lambda: self.k * self.x())
        self.var('damper', lambda: self.d * self.v())
        self.der('x', lambda: self.v())
        self.der('v', lambda: self.F() - self.damper() - self.spring())


class MSDAutonomous(Model):
    def __init__(self):
        super().__init__()
        self.state('x', 0.0)
        self.state('v', 1.0)

        self.parameter('k', 1.0)
        self.parameter('d', 1.0)
        # self.var('spring', lambda: self.k * self.x())
        # self.var('damper', lambda: self.d * self.v())
        self.der('x', lambda: self.v())
        self.der('v', lambda: - self.d * self.v() - self.k * self.x())

        self.save()

class TimeDepInput(Model):
    def __init__(self):
        super().__init__()
        self.var('F', lambda: 0.0 if self.time() < 4.0 else 4.0)


class DelayExample(Model):
    def __init__(self):
        super().__init__()
        self.input('u')
        self.var('d', lambda: self.u(-1.0))


class MSDTimeDep(Model):
    def __init__(self):
        super().__init__()

        self.model('msd', MassSpringDamperFlat())
        self.model('u', TimeDepInput())

        self.connect(self.msd, 'F', self.u, 'F')

        self.save()