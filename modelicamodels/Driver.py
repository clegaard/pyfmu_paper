class Driver:

    def __init__(self):
        super().__init__()

    @staticmethod
    def get_steering(width):
        def deltaf(t):
            amplitude = 1.2
            risingtime = 2.0
            startTime = 2.0
            if (t < startTime):
                return 0.0
            elif ((t - startTime) < risingtime):
                return amplitude * (t - startTime) / risingtime
            elif ((t - startTime - risingtime) < width):
                return amplitude
            else:
                return max(0.0, amplitude - amplitude * (t - startTime - risingtime - width) / risingtime)
        return deltaf


