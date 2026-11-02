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


def run(args, accepted=True, step_seconds=1 / 60):
    result = subprocess.run([binary, *args], env=env, capture_output=True, text=True, timeout=30)
    if accepted:
        assert result.returncode == 0, result.stderr
        assert "ODE Message" not in result.stderr and "ODE Error" not in result.stderr, result.stderr
        value = json.loads(result.stdout)
        assert value["success"] and value["substeps"] == 5
        assert abs(value["step_seconds"] - step_seconds) < 1e-12
        assert value["max_shell_penetration_m"] <= 0.002, value["max_shell_penetration_m"]
        return value
    assert result.returncode != 0, "invalid recording was accepted"


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
            variants = []
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
        def message(field, payload):
            assert len(payload) < 128
            return bytes([(field << 3) | 2, len(payload)]) + payload
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
