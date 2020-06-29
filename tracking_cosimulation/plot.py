import pandas as pd
import matplotlib.pyplot as plt

input = pd.read_csv("output.csv")

_, (p1, p2, p3, p4) = plt.subplots(1, 4)

p1.plot(input["time"], input["{src}.srci.deltaf"],
            label='steering')
p1.legend()
p2.plot(input['{bd}.bdi.X'], input['{bd}.bdi.Y'],
            label='dX vs dY')
p2.plot(input['{track}.tracki.X'], input['{track}.tracki.Y'], label='~dX vs ~dY')
# for calib in m.tracking.recalibration_history:
#     p2.plot(calib.xs[m.tracking.X_idx, :], calib.xs[m.tracking.Y_idx, :], '--', label='recalibration')
p2.legend()
p3.plot(input['time'], input['{track}.tracki.error'], label='error')
p3.plot(input['time'], input['{track}.tracki.tolerance'], label='tolerance')
p3.legend()
p4.plot(input['time'], input['{src}.srci.Caf'], label='real_Caf')
p4.plot(input['time'], input['{track}.tracki.Caf'], label='approx_Caf')
p4.legend()
plt.show()