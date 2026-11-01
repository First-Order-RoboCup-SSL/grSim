#!/usr/bin/env python3
"""Check production masks against unfiltered ODE contacts and full body traces."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def run(binary, scenario, trace, filtered):
    command = [str(binary), "--scenario", scenario, "--steps", "120",
               "--audit", "1", "--trace-file", str(trace)]
    # Omit --masks for the production-default run.
    if not filtered:
        command += ["--masks", "0"]
    result = subprocess.run(command, check=True, capture_output=True, text=True,
                            timeout=60, env={**os.environ, "QT_QPA_PLATFORM": "offscreen"})
    data = json.loads(result.stdout)
    if not data["success"] or not data["default_filtering_enabled"]:
        raise AssertionError(f"{scenario}: production filtering must be enabled by default")
    if data["masks"] != filtered:
        raise AssertionError(f"{scenario}: requested filtering mode was not applied")
    return data


def main():
    binary = Path(sys.argv[1]).resolve()
    # ConfigWidget resolves robot INI files relative to the executable.
    relative = "../config"
    if sys.platform.startswith("linux") and str(Path.home()) not in str(binary.parent):
        relative = "../share/grSim/config"
    if not (binary.parent / relative / "Parsian.ini").is_file():
        raise AssertionError("Missing binary-relative robot configuration")
    scenarios = ("spaced-6", "crowded-6", "pushing-6", "spaced-11", "spaced-16", "crowded-11", "crowded-16",
                 "idle-11", "stopstart-11", "ball-flight", "ball-rolling",
                 "ball-spin", "dribble-flat", "dribble-chip")
    fields = ("contact_hashes", "contact_counts", "narrow_phase_calls",
              "detected_contacts", "created_contacts", "max_penetration_m")
    with tempfile.TemporaryDirectory(prefix="grsim-collision-") as directory:
        traces = [Path(directory) / name for name in ("filtered.f64", "unfiltered.f64")]
        for scenario in scenarios:
            filtered = run(binary, scenario, traces[0], True)
            unfiltered = run(binary, scenario, traces[1], False)
            if scenario.endswith("-6") and filtered["diagnostics"]["max_planar_robot_overlap_m"] > 0.002:
                raise AssertionError(f"{scenario}: robot shells overlap by more than 2 mm")
            for field in fields:
                if filtered[field] != unfiltered[field]:
                    raise AssertionError(f"{scenario}: filtering changed {field}")
            actual, expected = (path.read_bytes() for path in traces)
            if not actual or actual != expected:
                raise AssertionError(f"{scenario}: full body trajectories differ")
            if filtered["mask_audit"]["registered_pairs"] != unfiltered["mask_audit"]["registered_pairs"]:
                raise AssertionError(f"{scenario}: surface graph changed")
            if not scenario.startswith("ball-") and filtered["callbacks"] >= unfiltered["callbacks"]:
                raise AssertionError(f"{scenario}: masks did not reduce callbacks")
            print(f"PASS {scenario}: identical contacts and trajectories, "
                  f"callbacks {unfiltered['callbacks']:.0f} -> {filtered['callbacks']:.0f}", flush=True)


if __name__ == "__main__":
    main()
