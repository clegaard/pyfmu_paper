import logging
from pathlib import Path
import subprocess
from tempfile import mkdtemp
import sys
import pandas as pd
import matplotlib.pyplot as plt


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__file__)


if __name__ == "__main__":

    jar_path = Path(__file__).parent / "coe.jar"
    config_path = Path(__file__).parent / "TrackingSimulator.json"
    result_path = Path(__file__).parent / "results.csv"

    assert jar_path.is_file()
    assert config_path.is_file()

    start_time = 0.0
    stop_time = 25.0

    args = [
        "java",
        "-jar",
        jar_path,
        "-v",
        "--configuration",
        str(config_path),
        "--oneshot",
        "--starttime",
        str(start_time),
        "--endtime",
        str(stop_time),
        "--result",
        str(result_path),
    ]
    results = subprocess.run(args)

    input = pd.read_csv(result_path)

    _, (p1, p2, p3, p4) = plt.subplots(1, 4)

    p1.plot(input["time"], input["{src}.srci.deltaf"], label="steering")
    p1.legend()
    p2.plot(input["{bd}.bdi.X"], input["{bd}.bdi.Y"], label="dX vs dY")
    if "{track}.tracki.X" in input.keys():
        p2.plot(
            input["{track}.tracki.X"], input["{track}.tracki.Y"], label="~dX vs ~dY"
        )
        # for calib in m.tracking.recalibration_history:
        #     p2.plot(calib.xs[m.tracking.X_idx, :], calib.xs[m.tracking.Y_idx, :], '--', label='recalibration')
        p2.legend()
    if "{track}.tracki.error" in input.keys():
        p3.plot(input["time"], input["{track}.tracki.error"], label="error")
        p3.plot(input["time"], input["{track}.tracki.tolerance"], label="tolerance")
        p3.legend()
        p4.plot(input["time"], input["{src}.srci.Caf"], label="real_Caf")
    if "{track}.tracki.Caf" in input.keys():
        p4.plot(input["time"], input["{track}.tracki.Caf"], label="approx_Caf")
        p4.legend()
        plt.savefig(fname="results.pdf", format="pdf")
        plt.show()

