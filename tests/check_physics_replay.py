"""Exercise actual recording/replay paths, including a live UDP capture."""
import base64
import copy
import json
import math
import os
from pathlib import Path
import re
import select
import socket
import struct
import subprocess
import sys
import tempfile
import time

binary = str(Path(sys.argv[1]).resolve())
env = dict(os.environ, QT_QPA_PLATFORM="offscreen")


def run(args, accepted=True, step_seconds=1 / 60, substeps=2):
    result = subprocess.run([binary, *args], env=env, capture_output=True, text=True, timeout=30)
    if accepted:
        assert result.returncode == 0, result.stderr
        assert "ODE Message" not in result.stderr and "ODE Error" not in result.stderr, result.stderr
        value = json.loads(result.stdout)
        assert value["success"] and value["substeps"] == substeps
        assert abs(value["step_seconds"] - step_seconds) < 1e-12
        assert value["max_shell_penetration_m"] <= 0.002, value["max_shell_penetration_m"]
        return value
    assert result.returncode != 0, "invalid recording was accepted"


def message(field, payload):
    length = len(payload)
    encoded = []
    while length > 127:
        encoded.append((length & 127) | 128)
        length >>= 7
    return bytes([(field << 3) | 2, *encoded, length]) + payload


def difference(left, right):
    assert len(left["checkpoints"]) == len(right["checkpoints"])
    maximum = 0
    for a, b in zip(left["checkpoints"], right["checkpoints"]):
        assert a["tick"] == b["tick"] and len(a["robots"]) == len(b["robots"])
        for ra, rb in zip(a["robots"], b["robots"]):
            assert ra["on"] == rb["on"] and ra["roller"] == rb["roller"]
            maximum = max(maximum, max(abs(x-y) for x,y in zip(ra["wheel_commands"], rb["wheel_commands"])))
        assert len(a["bodies"]) == len(b["bodies"])
        for ba, bb in zip(a["bodies"], b["bodies"]):
            for key in ba:
                for x, y in zip(ba[key], bb[key]):
                    assert math.isfinite(x) and math.isfinite(y)
                    maximum = max(maximum, abs(x - y))
    return maximum


with tempfile.TemporaryDirectory(prefix="grsim-replay-") as directory:
    directory = Path(directory)
    for settings in [directory / "missing.xml", directory]:
        output = directory / "invalid-settings-output.json"
        run(["--record", str(output), "--fixture", "spaced", "--ticks", "1",
             "--settings", str(settings)], accepted=False)
        assert not output.exists(), "invalid settings produced a recording"
    print("PASS missing/non-file explicit settings are rejected before recording", flush=True)
    settings = directory / "settings.xml"
    for contents in ["", '<VarXML><Var name="Physics" type="list">',
                     "<settings/>", '<VarXML xmlns="urn:unexpected"/>',
                     "<VarXML/><VarXML/>"]:
        settings.write_text(contents)
        output = directory / "malformed-settings-output.json"
        run(["--record", str(output), "--fixture", "spaced", "--ticks", "1",
             "--settings", str(settings)], accepted=False)
        assert not output.exists(), "malformed settings produced a recording"
    print("PASS malformed/wrong-root explicit settings are rejected before recording", flush=True)

    # A capture shorter than one valid physics tick can finish without stepping.
    # Keep the zero-tick case deterministic as well as checking the real timer.
    path = directory / "short.json"
    settings.write_text(
        '<?xml version="1.0"?>\n<!-- sparse replay settings -->\n'
        '<VarXML><Var name="Geometry" type="list"><Var name="Game" type="list">'
        '<Var name="Robots Count" type="int">6</Var></Var></Var>'
        '<Var name="Physics" type="list"><Var name="World" type="list">'
        '<Var name="ODE time step" type="double">0.1</Var></Var></Var></VarXML>')
    captured = run(["--record", str(path), "--duration-ms", "50",
                    "--settings", str(settings)], step_seconds=.1)
    replayed = run(["--replay", str(path)], step_seconds=.1)
    assert difference(captured, replayed) <= 1e-12
    document = json.loads(path.read_text())
    document["ticks"] = 0
    document["events"] = []
    path.write_text(json.dumps(document))
    empty = run(["--replay", str(path)], step_seconds=.1)
    assert empty["ticks"] == 0 and empty["contacts"] == 0
    assert empty["checkpoints"] == [document["initial"]]
    # A terminal tick-zero event must still be applied even without a step.
    teleport = b"\x0a\x07\x0a\x05\x0d" + struct.pack("<f", 2)
    document["events"] = [{"tick": 0, "kind": "simulator_control",
                           "data": base64.b64encode(teleport).decode("ascii")}]
    path.write_text(json.dumps(document))
    commanded = run(["--replay", str(path)], step_seconds=.1)
    assert commanded["ticks"] == 0 and commanded["contacts"] == 0
    assert commanded["checkpoints"][0]["bodies"][0]["position"][0] == 2
    assert commanded["checkpoints"][0]["simulation_time"] == document["initial"]["simulation_time"]
    print("PASS short and zero-tick capture timelines replay exactly, including terminal commands", flush=True)
    # Explicit teleport velocities keep their historical unscaled semantics.
    identity = b"\x08\x00\x10\x02"  # robot 0, BLUE
    velocity_fields = (b"\x2d" + struct.pack("<f", .1) + b"\x35" + struct.pack("<f", .2) +
                       b"\x3d" + struct.pack("<f", .3))
    teleport = message(1, message(2, message(1, identity) + velocity_fields))
    document["events"] = [{"tick": 0, "kind": "simulator_control",
                           "data": base64.b64encode(teleport).decode("ascii")}]
    path.write_text(json.dumps(document))
    unit_teleport = run(["--replay", str(path)], step_seconds=.1)
    document["settings"]["blueSettings"].update(
        ForwardVelocityScale=.962, LeftVelocityScale=.924,
        AngularVelocityScale=.958, AngularVelocityScaleSlope=.023)
    path.write_text(json.dumps(document))
    calibrated_teleport = run(["--replay", str(path)], step_seconds=.1)
    assert unit_teleport["checkpoints"][0]["robots"][0]["wheel_commands"] == calibrated_teleport["checkpoints"][0]["robots"][0]["wheel_commands"]
    print("PASS calibrated robot response does not alter teleport velocity targets", flush=True)
    profile_path = Path(__file__).resolve().parents[1] / "config" / "Utama-Field-20261028.xml"
    profile_recording = directory / "field-profile.json"
    run(["--record", str(profile_recording), "--fixture", "spaced", "--ticks", "30",
         "--settings", str(profile_path)])
    field_settings = json.loads(profile_recording.read_text())["settings"]
    assert field_settings["BlueTeam"] == "Utama-Radio-1" and field_settings["YellowTeam"] == "Utama-Radio-0"
    assert field_settings["BallRollingDeceleration"] == .2
    for name, expected in [("blueSettings", (.962, .924, .958, .023)), ("yellowSettings", (.974, .958, 1.002, .021))]:
        assert tuple(field_settings[name][key] for key in
                     ["ForwardVelocityScale", "LeftVelocityScale", "AngularVelocityScale", "AngularVelocityScaleSlope"]) == expected
    print("PASS physical robot profiles and measured ball settings load and record explicitly", flush=True)
    fixed_document = json.loads(profile_recording.read_text())
    fixed_document["integration_policy"] = "fixed-v1"
    fixed_results = {}
    fixed_path = directory / "fixed-step-count.json"
    for count in [1, 2, 3, 5]:
        fixed_document["substeps"] = count
        fixed_path.write_text(json.dumps(fixed_document))
        fixed_results[count] = run(["--replay", str(fixed_path)], substeps=count)
        assert fixed_results[count]["integration_steps"] == fixed_results[count]["ticks"] * count
    assert difference(fixed_results[1], fixed_results[5]) > 1e-8
    for count in [0, 33]:
        fixed_document["substeps"] = count
        fixed_path.write_text(json.dumps(fixed_document))
        run(["--replay", str(fixed_path)], accepted=False)
    print("PASS explicit fixed step counts alter integration without changing the tick clock", flush=True)
    def floats(*values):
        return b"".join(bytes([(field << 3) | 5]) + struct.pack("<f", value)
                        for field, value in enumerate(values, 1))

    def input_event(tick, kind, payload, team=None):
        event = dict(tick=tick, kind=kind, data=base64.b64encode(payload).decode("ascii"))
        if team is not None:
            event["team"] = team
        return event

    # Keep twelve robots present, with the other eleven outside each shot path.
    # A late teleport also exercises settling after the initial warmup.
    wall_y = (field_settings["Field_Width"] / 2 + field_settings["Field_Margin_Touch_Line"] +
              field_settings["Field_Referee_Margin"])
    for case in ["ball-back", "ball-front", "ball-wall", "robot-wall", "far-cruise"]:
        target = (0, wall_y - .5, math.pi / 2) if case == "robot-wall" else (0, 0, 0)
        if case == "far-cruise":
            target = (-2, 0, 0)
        control = message(1, floats(4, -3, 0, 0, 0, 0))
        for team in range(2):
            for robot in range(6):
                pose = target if (team, robot) == (0, 0) else (-4 + robot * .5, 2 + team * .5, 0)
                identity = bytes([8, robot, 16, 2 if team == 0 else 1])
                control += message(2, message(1, identity) + b"\x15" + struct.pack("<f", pose[0]) +
                                   b"\x1d" + struct.pack("<f", pose[1]) +
                                   b"\x25" + struct.pack("<f", pose[2]))
        impact = copy.deepcopy(json.loads(profile_recording.read_text()))
        impact["ticks"] = 180
        impact["events"] = [input_event(0, "simulator_control", message(1, control))]
        if case.startswith("ball"):
            shot = {"ball-back": (-.6, 0, 0, 2.2, 0, 0),
                    "ball-front": (.6, 0, 0, -2.2, 0, 0),
                    "ball-wall": (0, wall_y - .5, 0, 0, 2.2, 0)}[case]
            # z=0 uses the existing teleport height: radius plus 5 mm.
            impact["events"].append(input_event(90, "simulator_control", message(1, message(1, floats(*shot)))))
        else:
            speed = 1 if case == "robot-wall" else .5
            command = message(1, b"\x08\x00" + message(2, message(2, floats(speed, 0, 0))))
            impact["events"] += [input_event(tick, "robot_control", command, 0) for tick in range(60, 180, 3)]
        impact_path = directory / (case + ".json")
        impact_path.write_text(json.dumps(impact))
        serial = run(["--replay", str(impact_path), "--checkpoint-every", "1"])
        workers = run(["--replay", str(impact_path), "--helpers", "3", "--checkpoint-every", "1"])
        repeated = run(["--replay", str(impact_path), "--checkpoint-every", "1"])
        assert difference(serial, repeated) <= 1e-12
        assert difference(serial, workers) <= 1e-6
        for value in [serial, workers]:
            assert value["max_ball_solid_penetration_m"] <= .002
            assert value["max_robot_wall_penetration_m"] <= .002
        if case.startswith("ball"):
            assert serial["max_ball_solid_penetration_m"] > 0
            if case == "ball-wall":
                assert min(p["bodies"][0]["velocity"][1] for p in serial["checkpoints"][90:]) < -.1
        elif case == "robot-wall":
            assert serial["max_robot_wall_penetration_m"] > 0
            assert serial["checkpoints"][-1]["bodies"][1]["position"][1] < wall_y
        else:
            assert serial["integration_steps"] < 180 * 4
            assert serial["checkpoints"][-1]["bodies"][1]["position"][0] > -1.2
        print(f"PASS {case}: bounded impacts, serial repeat and worker agreement", flush=True)

    # Both observed turn speeds must retain the 2 mm bound, including the
    # previously failing 4 rad/s first-impact case.
    for fixture in ["pushing-turning", "pushing-turning-fast"]:
        turning_path = directory / (fixture + ".json")
        turning = run(["--record", str(turning_path), "--fixture", fixture,
                       "--ticks", "900", "--settings", str(profile_path)])
        turning_workers = run(["--replay", str(turning_path), "--helpers", "3"])
        turning_repeated = run(["--replay", str(turning_path), "--helpers", "0"])
        assert difference(turning, turning_repeated) <= 1e-12
        short_serial = dict(turning, checkpoints=[p for p in turning["checkpoints"] if p["tick"] <= 180])
        short_workers = dict(turning_workers, checkpoints=[p for p in turning_workers["checkpoints"] if p["tick"] <= 180])
        assert difference(short_serial, short_workers) <= 1e-6
        assert turning["integration_policy"] == "adaptive-v1"
        assert turning["integration_steps"] > turning["ticks"] * 5
        print(f"PASS 6v6 {fixture}: <2mm contacts, exact serial repeat and short worker agreement", flush=True)
    for fixture in ["spaced", "pushing"]:
        path = directory / (fixture + ".json")
        recorded = run(["--record", str(path), "--fixture", fixture, "--ticks", "180"])
        serial = run(["--replay", str(path), "--helpers", "0"])
        repeated = run(["--replay", str(path), "--helpers", "0"])
        workers = run(["--replay", str(path), "--helpers", "3"])
        assert difference(recorded, serial) <= 1e-12
        assert difference(serial, repeated) <= 1e-12
        delta = difference(serial, workers)
        assert delta <= 1e-6, delta
        print(f"PASS {fixture}: recorded/repeated serial <=1e-12, worker state delta {delta:.3g}", flush=True)
        document = json.loads(path.read_text())
        assert document["settings"]["Robots_Count"] == 6
        assert document["initial"]["tick"] == 0 and document["seed"] == 166
        if fixture == "spaced":
            dense = run(["--replay", str(path), "--checkpoint-every", "1"])
            assert [point["tick"] for point in dense["checkpoints"]] == list(range(181))
            sampled = dict(dense, checkpoints=[point for point in dense["checkpoints"] if point["tick"] % 30 == 0])
            assert difference(serial, sampled) <= 1e-12
            for interval in ["0", "1000001"]:
                run(["--replay", str(path), "--checkpoint-every", interval], accepted=False)
            print("PASS per-tick physics snapshots preserve the original replay trajectory", flush=True)
            dense_path = directory / "dense-recording.json"
            dense_record = run(["--record", str(dense_path), "--fixture", "spaced",
                                "--ticks", "180", "--checkpoint-every", "1"])
            assert json.loads(dense_path.read_text())["events"] == document["events"]
            sampled_record = dict(dense_record, checkpoints=[point for point in dense_record["checkpoints"] if point["tick"] % 30 == 0])
            assert difference(recorded, sampled_record) <= 1e-12
            print("PASS recording snapshot density leaves fixture commands and physics unchanged", flush=True)
            delayed = copy.deepcopy(document)
            for event in delayed["events"]:
                if event["kind"] == "robot_control" and event["tick"] == 0:
                    event["tick"] = 3
            changed = directory / "delayed.json"
            changed.write_text(json.dumps(delayed))
            altered = run(["--replay", str(changed)])
            # Compare physical states separately: wheel command histories can
            # legitimately differ when a command's accepted tick changes.
            delta = max(abs(x-y) for a,b in zip(serial["checkpoints"], altered["checkpoints"])
                        for ba,bb in zip(a["bodies"],b["bodies"])
                        for key in ["position", "velocity"]
                        for x,y in zip(ba[key],bb[key]))
            assert delta > 1e-4, "changing accepted command ticks had no detectable effect"
            print("PASS negative control detects shifted command ticks", flush=True)
            legacy = copy.deepcopy(document)
            for model in ["robotSettings", "blueSettings", "yellowSettings"]:
                for key in ["ForwardVelocityScale", "LeftVelocityScale", "AngularVelocityScale", "AngularVelocityScaleSlope"]:
                    del legacy["settings"][model][key]
            changed.write_text(json.dumps(legacy))
            assert difference(serial, run(["--replay", str(changed)])) <= 1e-12
            print("PASS omitted response fields retain exact unit-response physics", flush=True)
            fixed = copy.deepcopy(legacy)
            fixed["integration_policy"] = "fixed-5-v1"
            fixed["substeps"] = 5
            changed.write_text(json.dumps(fixed))
            fixed_result = run(["--replay", str(changed)], substeps=5)
            del fixed["integration_policy"]
            fixed["format"] = "grsim-physics-replay-v1"
            changed.write_text(json.dumps(fixed))
            historical = run(["--replay", str(changed)], substeps=5)
            assert historical["integration_policy"] == "fixed-5-v1"
            assert historical["integration_steps"] == historical["ticks"] * 5
            assert difference(fixed_result, historical) <= 1e-12
            print("PASS recordings without integration metadata retain fixed-five physics", flush=True)
            variants = []
            unknown_policy = copy.deepcopy(document); unknown_policy["integration_policy"] = "unknown"
            variants.append(unknown_policy)
            missing_policy = copy.deepcopy(document); del missing_policy["integration_policy"]
            variants.append(missing_policy)
            wrong_legacy_policy = copy.deepcopy(document); wrong_legacy_policy["format"] = "grsim-physics-replay-v1"
            variants.append(wrong_legacy_policy)
            for key, value in [("ForwardVelocityScale", 0), ("LeftVelocityScale", -1),
                               ("AngularVelocityScale", "not numeric"), ("AngularVelocityScaleSlope", -1)]:
                invalid_gain = copy.deepcopy(document)
                invalid_gain["settings"]["blueSettings"][key] = value
                variants.append(invalid_gain)
            wrong_format = copy.deepcopy(document); wrong_format["format"] = "unknown"
            variants.append(wrong_format)
            missing = copy.deepcopy(document); del missing["settings"]["BallMass"]
            variants.append(missing)
            unknown = copy.deepcopy(document); unknown["events"][0]["kind"] = "unknown"
            variants.append(unknown)
            payload = copy.deepcopy(document); payload["events"][0]["data"] = "not base64!"
            variants.append(payload)
            unordered = copy.deepcopy(document); unordered["events"][0]["tick"] = 180
            variants.append(unordered)
            invalid = copy.deepcopy(document); invalid["settings"]["DeltaTime"] = -1
            variants.append(invalid)
            wrong_initial = copy.deepcopy(document)
            wrong_initial["initial"]["bodies"][0]["position"][0] += .001
            variants.append(wrong_initial)
            for index, value in enumerate(variants):
                changed.write_text(json.dumps(value))
                run(["--replay", str(changed)], accepted=False)
            print("PASS malformed format/settings/payload/timeline controls are rejected", flush=True)
    path = directory / "live.json"
    process = subprocess.Popen([binary, "--record", str(path), "--duration-ms", "1700"],
                               env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        ready = ""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if select.select([process.stderr], [], [], .1)[0]:
                line = process.stderr.readline()
                if line.startswith("READY "):
                    ready = line
                    break
            if process.poll() is not None:
                break
        assert ready, "capture did not announce its loopback ports"
        port = int(re.search(r"blue=(\d+)", ready).group(1))
        simulator_port = int(re.search(r"simulator=(\d+)", ready).group(1))
        # Reject the whole batch before applying its ball or robot teleports,
        # including explicitly present false-valued unsupported flags.
        identity = b"\x08\x00\x10\x02"  # robot 0, BLUE
        teleport_robot = message(1, identity) + b"\x15" + struct.pack("<f", 4)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as simulator:
            for field in [7, 8]:  # teleport_safely, roll
                for enabled in [False, True]:
                    teleport_ball = b"\x0d" + struct.pack("<f", 2) + bytes([field << 3, int(enabled)])
                    control = message(1, teleport_ball) + message(2, teleport_robot)
                    simulator.sendto(message(1, control), ("127.0.0.1", simulator_port))
        # Minimal valid RobotControl/RobotCommand/MoveLocalVelocity wire packet.
        # Required float fields use protobuf fixed32 little-endian wire type 5.
        velocity = b"\x0d" + struct.pack("<f", .2) + b"\x15" + struct.pack("<f", 0) + b"\x1d" + struct.pack("<f", 0)
        move = b"\x12" + bytes([len(velocity)]) + velocity
        robot = b"\x08\x00\x12" + bytes([len(move)]) + move
        packet = b"\x0a" + bytes([len(robot)]) + robot
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.settimeout(2)
            for _ in range(3):
                sender.sendto(packet, ("127.0.0.1", port))
                response, _ = sender.recvfrom(4096)
                assert response, "capture omitted command feedback"
                time.sleep(.04)
        output, errors = process.communicate(timeout=15)
        assert process.returncode == 0, errors
        recorded = json.loads(output)
        replayed = run(["--replay", str(path)])
        assert difference(recorded, replayed) <= 1e-12
        document = json.loads(path.read_text())
        assert errors.count("Rejected capture command:") == 4, errors
        assert not any(event["kind"] == "simulator_control" for event in document["events"])
        assert any(event["kind"] == "robot_control" for event in document["events"])
        assert any(event["kind"] == "expire" and event["team"] == 0 for event in document["events"])
        print("PASS rejected teleport batches leave state unchanged; live UDP feedback/expiry replay exactly", flush=True)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
