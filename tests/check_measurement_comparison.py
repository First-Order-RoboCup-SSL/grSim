"""Independent arithmetic/negative controls for measurement comparison."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import sys
sys.dont_write_bytecode = True

path = Path(__file__).resolve().parent.parent / "tools/compare_ball_measurements.py"
spec = importlib.util.spec_from_file_location("comparison", path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
# Deliberately synthetic, analytically known traces test the comparator only.
# These are not passed off as measured or used to calibrate physics defaults.
document = {"success":True, "trajectory_columns":["t_s","ball_x_m","ball_y_m","ball_z_m",
    "ball_vx_m_s","ball_vy_m_s","ball_wx_rad_s","ball_wy_rad_s"],
    "trajectory":[[i, 2*i, -i, .02, 2, -1, 0, 0] for i in range(5)]}
with tempfile.TemporaryDirectory(prefix="grsim-measurements-") as directory:
    path = Path(directory) / "samples.csv"
    def write(rows):
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["t_s","x_m","y_m"])
            writer.writerows(rows)
    samples = [[t, 2*t + .03, -t - .04] for t in [.25, 1.25, 2.25, 3.25]]
    write(samples)
    result = module.compare_csv(document, path)
    assert abs(result["xy_rmse_m"] - .05) < 1e-12
    assert abs(result["xy_max_error_m"] - .05) < 1e-12
    for rows in [samples[:2], list(reversed(samples)), samples[:3]+[samples[2]],
                 samples[:3]+[[10,0,0]], samples[:3]+[[float("nan"),0,0]]]:
        write(rows)
        try:
            module.compare_csv(document, path)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid/misaligned measurements were accepted")
print("PASS independent 3-4-5 error/interpolation, missing/duplicate/unordered/nonfinite/out-of-range measurement controls")
