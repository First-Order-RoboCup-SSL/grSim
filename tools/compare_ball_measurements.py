#!/usr/bin/env python3
"""Compare an actual grSim ball trajectory with SI-unit CSV samples or a published coefficient reference."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics


def trajectory(document):
    if not document.get("success"):
        raise ValueError("simulation did not succeed")
    columns = document["trajectory_columns"]
    names = ["t_s", "ball_x_m", "ball_y_m", "ball_z_m", "ball_vx_m_s",
             "ball_vy_m_s", "ball_wx_rad_s", "ball_wy_rad_s"]
    indices = [columns.index(name) for name in names]
    rows = [[float(row[i]) for i in indices] for row in document["trajectory"]]
    if len(rows) < 4 or any(not math.isfinite(x) for row in rows for x in row):
        raise ValueError("insufficient or nonfinite simulated samples")
    if any(a[0] >= b[0] for a, b in zip(rows, rows[1:])):
        raise ValueError("simulation timestamps must be strictly increasing")
    return rows


def compare_csv(document, filename, time_offset=0):
    rows = trajectory(document)
    with Path(filename).open(newline="") as handle:
        reader = csv.DictReader(handle)
        if not {"t_s", "x_m", "y_m"}.issubset(reader.fieldnames or []):
            raise ValueError("CSV needs t_s,x_m,y_m in seconds and metres")
        has_z = "z_m" in reader.fieldnames
        measured = [[float(row[key]) for key in ["t_s", "x_m", "y_m"] + (["z_m"] if has_z else [])]
                    for row in reader]
    if len(measured) < 4 or any(not math.isfinite(x) for row in measured for x in row):
        raise ValueError("need at least four finite measurement rows")
    if any(a[0] >= b[0] for a, b in zip(measured, measured[1:])):
        raise ValueError("measurement timestamps must be strictly increasing; select one camera/track first")
    times = [row[0] for row in rows]
    errors, errors_z = [], []
    for sample in measured:
        t = sample[0] + time_offset
        if t < times[0] or t > times[-1]:
            raise ValueError("measurement extends outside simulation; extrapolation is disabled")
        index = min(max(1, bisect.bisect_left(times, t)), len(rows) - 1)
        a, b = rows[index - 1], rows[index]
        fraction = (t - a[0]) / (b[0] - a[0])
        predicted = [a[i] + fraction * (b[i] - a[i]) for i in range(1, 4)]
        errors.append(math.hypot(predicted[0] - sample[1], predicted[1] - sample[2]))
        if has_z:
            errors_z.append(abs(predicted[2] - sample[3]))
    return {"comparison":"measured trajectory samples", "measurement_rows":len(measured),
            "time_offset_s":time_offset, "automatic_fitting":False,
            "xy_rmse_m":math.sqrt(statistics.mean(x*x for x in errors)),
            "xy_max_error_m":max(errors),
            "z_rmse_m":math.sqrt(statistics.mean(x*x for x in errors_z)) if errors_z else None}


def compare_reference(document, reference):
    rows = trajectory(document)
    radius = float(document["parameters"]["ball_radius_m"])
    if radius <= 0 or not math.isfinite(radius):
        raise ValueError("invalid ball radius")
    sliding, rolling = [], []
    initial_speed = math.hypot(rows[0][4], rows[0][5])
    transition = None
    was_sliding = False
    for a, b in zip(rows, rows[1:]):
        if abs(a[3] - radius) > .001 or abs(b[3] - radius) > .001:
            continue
        slip_a = math.hypot(a[4] - radius*a[7], a[5] + radius*a[6])
        slip_b = math.hypot(b[4] - radius*b[7], b[5] + radius*b[6])
        acceleration = (math.hypot(b[4], b[5]) - math.hypot(a[4], a[5])) / (b[0] - a[0])
        if slip_a > .03 and slip_b > .03:
            sliding.append(acceleration)
            was_sliding = True
        if slip_a < .01 and slip_b < .01 and b[0] > .2 and math.hypot(b[4], b[5]) > .1:
            rolling.append(acceleration)
        if was_sliding and transition is None and slip_b < .01 and initial_speed > 0:
            transition = math.hypot(b[4], b[5]) / initial_speed
    observed = {"sliding_acceleration_m_s2":statistics.median(sliding) if sliding else None,
                "rolling_acceleration_m_s2":statistics.median(rolling) if rolling else None,
                "switch_speed_ratio":transition}
    metrics = {}
    for name, value in observed.items():
        expected = float(reference[name])
        tolerance = float(reference["comparison_tolerances"][name])
        metrics[name] = {"observed":value, "reference":expected, "tolerance":tolerance,
                         "within_tolerance":value is not None and abs(value-expected) <= tolerance,
                         "absolute_error":abs(value-expected) if value is not None else None}
    return {"comparison":"published fitted coefficients, not raw measurement samples",
            "reference":reference["reference"], "source_url":reference["source_url"],
            "scope":reference["scope"], "tolerance_note":reference["tolerance_note"],
            "agreement":all(value["within_tolerance"] for value in metrics.values()),
            "metrics":metrics}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulation_json", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--measured-csv", type=Path)
    group.add_argument("--reference-json", type=Path)
    parser.add_argument("--time-offset-s", type=float, default=0)
    parser.add_argument("--require-agreement", action="store_true",
                        help="fail on published-reference mismatch; CSV comparisons report errors without inventing an acceptance threshold")
    args = parser.parse_args()
    try:
        if not math.isfinite(args.time_offset_s):
            raise ValueError("time offset must be finite")
        if args.require_agreement and not args.reference_json:
            raise ValueError("--require-agreement requires a coefficient reference")
        document = json.loads(args.simulation_json.read_text())
        result = (compare_reference(document, json.loads(args.reference_json.read_text()))
                  if args.reference_json else compare_csv(document, args.measured_csv, args.time_offset_s))
        print(json.dumps(result, indent=2, allow_nan=False))
        return 1 if args.require_agreement and not result["agreement"] else 0
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
