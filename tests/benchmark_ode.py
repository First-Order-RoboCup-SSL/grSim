#!/usr/bin/env python3
"""Compare the headless ODE benchmark modes using interleaved repeated runs."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import random
import statistics
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1 or args.steps < 1:
        parser.error("repeats and steps must be positive")
    modes = [("current-serial", args.executable.resolve(), 0),
             ("current-2-helpers", args.executable.resolve(), 2),
             ("current-4-helpers", args.executable.resolve(), 4)]
    if args.baseline:
        modes.insert(0, ("baseline", args.baseline.resolve(), 0))
    cases = [(mode, executable, helpers, robots, crowded)
             for mode, executable, helpers in modes
             for robots, crowded in [(11, False), (16, False), (11, True)]]
    jobs = []
    randomizer = random.Random(166)
    for measurement in range(1, args.repeats + 1):
        round_cases = cases.copy()
        randomizer.shuffle(round_cases)
        jobs.extend((measurement, *case) for case in round_cases)
    started_at = datetime.now(timezone.utc).isoformat()
    results = []
    for measurement, mode, executable, helpers, robots, crowded in jobs:
        command = [str(executable), "--benchmark", "--threads", str(helpers),
                   "--robots", str(robots), "--steps", str(args.steps)]
        if crowded:
            command.append("--crowded")
        result = json.loads(subprocess.check_output(
            command, env=dict(os.environ, QT_QPA_PLATFORM="offscreen"), text=True, timeout=120))
        result["mode"] = mode
        result["measurement_round"] = measurement
        results.append(result)
        print(f"{len(results)}/{len(jobs)} {mode} robots/team={robots} "
              f"crowded={crowded} ms/tick={result['ms_per_tick']:.3f}", flush=True)
    summaries = []
    for mode, _, helpers in modes:
        for robots, crowded in [(11, False), (16, False), (11, True)]:
            samples = [r for r in results if r["mode"] == mode and
                       r["robots_per_team"] == robots and r["crowded"] == crowded]
            serial = next(r for r in results if r["mode"] == "current-serial" and
                          r["robots_per_team"] == robots and r["crowded"] == crowded)
            if any(len(r["state"]) != len(serial["state"]) for r in samples):
                raise ValueError("Benchmark state shapes differ between builds")
            times = [r["ms_per_tick"] for r in samples]
            summaries.append({
                "mode": mode, "ode_version": samples[0]["ode_version"], "helpers": helpers,
                "robots_per_team": robots, "crowded": crowded,
                "sample_count": len(times),
                "mean_ms_per_tick": statistics.mean(times),
                "stdev_ms_per_tick": statistics.stdev(times) if len(times) > 1 else 0.0,
                "median_ms_per_tick": statistics.median(times),
                "min_ms_per_tick": min(times), "max_ms_per_tick": max(times),
                "max_state_difference_from_current_serial": max(
                    max(abs(a-b) for a, b in zip(r["state"], serial["state"])) for r in samples),
                "configuration": samples[0]["configuration"],
            })
    for result in results:
        del result["state"]
    cpu_model = platform.processor()
    if Path("/proc/cpuinfo").exists():
        cpu_model = next((line.partition(":")[2].strip() for line in
                          Path("/proc/cpuinfo").read_text().splitlines()
                          if line.startswith("model name")), cpu_model)
    report = {"cpu": cpu_model, "logical_cpus": os.cpu_count(),
              "platform": platform.platform(), "steps_per_sample": args.steps,
              "warmup_ticks": 60, "repeats": args.repeats,
              "started_at_utc": started_at,
              "completed_at_utc": datetime.now(timezone.utc).isoformat(),
              "execution_order": "Shuffled sequential cases within each measurement round; seed 166.",
              "scope": "Collision detection including default ball-ground contacts, dWorldStep and robot motors; excludes rendering, networking, snapshots and real-time throttling.",
              "summaries": summaries, "samples": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
