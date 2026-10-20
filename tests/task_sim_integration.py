#!/usr/bin/env python3
"""Exercise task_sim + Foxglove without Bevy, OpenVINO, or Python packages.

Usage: python3 tests/task_sim_integration.py ./build/task_sim
The nine second projected-armor feed includes rotation, a stop, and reversal.
It exercises Planner -> synthetic gimbal feedback -> Shooter launch requests.
Synthetic launches verify control plumbing; hit_count must remain zero because
this feed does not implement collision or ballistic hit detection.
"""

import base64
import json
import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time


class WebSocket:
    def __init__(self, port):
        self.socket = socket.create_connection(("127.0.0.1", port), timeout=3)
        self.buffer = b""
        key = base64.b64encode(os.urandom(16)).decode()
        self.socket.sendall((
            f"GET / HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n\r\n").encode())
        while b"\r\n\r\n" not in self.buffer:
            data = self.socket.recv(65536)
            if not data:
                raise EOFError("connection closed during handshake")
            self.buffer += data
        header, self.buffer = self.buffer.split(b"\r\n\r\n", 1)
        assert b"101 Switching Protocols" in header, header
        assert b"foxglove.websocket.v1" in header, header

    def exact(self, size):
        while len(self.buffer) < size:
            data = self.socket.recv(max(4096, size - len(self.buffer)))
            if not data:
                raise EOFError()
            self.buffer += data
        data, self.buffer = self.buffer[:size], self.buffer[size:]
        return data

    def send(self, opcode, payload):
        mask = os.urandom(4)
        length = len(payload)
        header = bytes([128 | opcode])
        header += (bytes([128 | length]) if length < 126
                   else bytes([128 | 126]) + struct.pack("!H", length))
        body = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        self.socket.sendall(header + mask + body)

    def message(self):
        chunks, initial_opcode = [], None
        while True:
            first, second = self.exact(2)
            final, opcode, length = first & 128, first & 15, second & 127
            if length == 126:
                length = struct.unpack("!H", self.exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self.exact(8))[0]
            assert length < 16 * 1024 * 1024, "unbounded server message"
            mask = self.exact(4) if second & 128 else None
            body = self.exact(length)
            if mask:
                body = bytes(c ^ mask[i % 4] for i, c in enumerate(body))
            if opcode == 8:
                raise EOFError()
            if opcode == 9:
                self.send(10, body)
                continue
            if opcode == 10:
                continue
            if initial_opcode is None:
                initial_opcode = opcode
            chunks.append(body)
            if final:
                return initial_opcode, b"".join(chunks)


def jpeg_dimensions(data):
    assert data[:2] == b"\xff\xd8" and data[-2:] == b"\xff\xd9", "invalid JPEG markers"
    offset = 2
    while offset + 4 <= len(data):
        assert data[offset] == 255, "invalid JPEG segment"
        marker = data[offset + 1]
        offset += 2
        length = struct.unpack("!H", data[offset:offset + 2])[0]
        if marker in {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
                      0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}:
            height, width = struct.unpack("!HH", data[offset + 3:offset + 7])
            return width, height
        offset += length
    raise AssertionError("JPEG lacks frame dimensions")


FIRE_REASONS = {
    "disabled", "control_disabled", "not_following", "not_tracking", "not_converged",
    "no_measurement", "stale_frame", "invalid_plan", "planner_hold", "plate_not_visible",
    "aim_not_aligned", "settling", "cooldown", "clock_reset", "fire",
}


def validate_control_sample(sample, previous):
    """Check the externally observable launch contract, not Planner internals."""
    for key in ["frame_age_ms", "processing_ms", "command_yaw", "command_pitch",
                "command_age_ms", "fire_yaw_error_deg", "fire_pitch_error_deg",
                "fire_cooldown_ms", "max_frame_age_ms", "fire_interval_ms"]:
        value = sample[key]
        assert isinstance(value, (int, float)) and not isinstance(value, bool) \
            and math.isfinite(value), f"invalid metric {key}: {value!r}"
    for key in ["planned_control", "solver_ok", "planner_fire", "fire", "following",
                "control_enabled", "fire_enabled", "tracking", "converged", "command_sent"]:
        assert isinstance(sample[key], bool), f"invalid boolean metric {key}"
    assert sample["max_frame_age_ms"] > 0, "invalid frame freshness limit"
    assert sample["fire_interval_ms"] >= 50, "unsafe or invalid launch interval"
    assert sample["fire_cooldown_ms"] >= 0, "negative cooldown"
    reason = sample["fire_reason"]
    assert reason in FIRE_REASONS, f"unknown fire reason {reason!r}"
    assert sample["fire"] == (reason == "fire"), "fire pulse and reason disagree"
    if sample["planned_control"]:
        assert sample["solver_ok"], "invalid Planner result advertised as usable"
    if sample["fire"]:
        for key in ["tracking", "converged", "following", "control_enabled", "fire_enabled",
                    "planned_control", "solver_ok", "planner_fire", "command_sent"]:
            assert sample[key], f"launch bypassed {key} gate on frame {sample['frame_seq']}"
        assert sample.get("accepted", 0) > 0, "launch without an accepted current measurement"
        assert 0 <= sample["command_age_ms"] <= sample["max_frame_age_ms"], \
            "launch used a stale frame"
        for axis in ["yaw", "pitch"]:
            tolerance = sample.get(f"fire_{axis}_tolerance_deg")
            if tolerance is not None:
                assert abs(sample[f"fire_{axis}_error_deg"]) <= tolerance + 1e-6, \
                    f"launch bypassed actual {axis} alignment"
    if not sample["following"] or not sample["control_enabled"] or not sample["fire_enabled"] \
            or not sample["solver_ok"] or not sample["tracking"] or not sample["converged"] \
            or sample.get("accepted", 0) == 0 \
            or sample["command_age_ms"] > sample["max_frame_age_ms"]:
        assert not sample["fire"], "closed loop launched through an invalid control gate"
    for key in ["shots_requested", "launch_count", "hit_count"]:
        assert isinstance(sample[key], int) and not isinstance(sample[key], bool) \
            and sample[key] >= 0, f"invalid counter {key}"
    assert sample["hit_count"] <= sample["launch_count"] <= sample["shots_requested"], \
        "launch/hit counters exceed issued requests"
    assert sample["launch_count"] == sample["shots_requested"], \
        "synthetic launch request did not reach the simulated actuator"
    assert sample["hit_count"] == 0, "synthetic stream claimed an unverified hit"
    if previous is not None:
        assert sample["frame_seq"] > previous["frame_seq"], "out-of-order debug frame"
        for key in ["shots_requested", "launch_count", "hit_count"]:
            assert sample[key] >= previous[key], f"{key} counter moved backwards"
        if sample["fire"]:
            assert sample["shots_requested"] > previous["shots_requested"], \
                "fire pulse did not increase the request counter"


def validate_angle_sample(sample):
    for name in ["target", "gimbal", "plan", "error"]:
        assert isinstance(sample[name]["valid"], bool), f"missing {name}.valid"
    assert sample["plan"]["valid"] == sample["planned_control"]
    assert sample["target"]["valid"] == sample["plan"]["valid"]
    for name in ["target", "gimbal", "plan"]:
        data = sample[name]
        if data["valid"]:
            for key in ["yaw", "pitch", "yaw_deg", "pitch_deg", "yaw_unwrapped", "yaw_unwrapped_deg"]:
                assert math.isfinite(data[key]), f"invalid {name}.{key}"
            assert abs(data["yaw_deg"] - math.degrees(data["yaw"])) < 1e-6
            assert abs(data["pitch_deg"] - math.degrees(data["pitch"])) < 1e-6
            assert abs(math.remainder(data["yaw_unwrapped"] - data["yaw"], 2 * math.pi)) < 1e-6
        else:
            assert "yaw" not in data and "pitch" not in data, "invalid plan fabricated zero angles"
    if sample["error"]["valid"]:
        error = math.remainder(sample["target"]["yaw"] - sample["gimbal"]["yaw"], 2 * math.pi)
        assert abs(sample["error"]["yaw"] - error) < 1e-6
    if sample["target"]["state_valid"]:
        assert abs(sample["target"]["phase"] - sample["yaw"]) < 1e-6
        assert abs(sample["target"]["phase_rate"] - sample["vyaw"]) < 1e-6


def run(executable):
    # Keep integration separate from an actual simulator's Talos consumer.
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("127.0.0.1", 0))
    udp.setblocking(False)
    udp_port = udp.getsockname()[1]
    udp_samples = []
    with tempfile.TemporaryFile(mode="w+t") as log:
        process = subprocess.Popen(
            [executable, "--synthetic", "--duration", "9", "--foxglove-port", str(port),
             "--plotjuggler-port", str(udp_port)],
            stdout=log, stderr=subprocess.STDOUT)
        client = None
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                try:
                    client = WebSocket(port)
                    break
                except ConnectionRefusedError:
                    if process.poll() is not None:
                        raise RuntimeError("task_sim exited before Foxglove connected")
                    time.sleep(0.02)
            assert client is not None, "Foxglove listener not available"
            opcode, body = client.message()
            assert opcode == 1 and json.loads(body)["op"] == "serverInfo"
            opcode, body = client.message()
            advertised = json.loads(body)
            assert opcode == 1 and advertised["op"] == "advertise"
            assert len(advertised["channels"]) == 3
            schema = json.loads(advertised["channels"][0]["schema"])
            assert schema["properties"]["target"]["properties"]["yaw_unwrapped_deg"]["type"] == "number"
            assert schema["properties"]["gimbal"]["properties"]["yaw"]["type"] == "number"
            client.send(1, json.dumps({"op": "subscribe", "subscriptions": [
                {"id": 41 + i, "channelId": 1 + i} for i in range(3)
            ]}, separators=(",", ":")).encode())
            samples, images, scenes = [], 0, 0
            while True:
                try:
                    opcode, body = client.message()
                except EOFError:
                    break
                while True:
                    try:
                        packet = udp.recv(65535)
                    except BlockingIOError:
                        break
                    curve = json.loads(packet)
                    assert len(packet) < 60000 and "image" not in curve and "ekf_x" not in curve
                    assert math.isfinite(curve["timestamp"]) and curve["time"] >= 0
                    assert all(name in curve for name in ["target", "gimbal", "plan", "error"])
                    if udp_samples:
                        assert curve["time"] > udp_samples[-1]["time"]
                    udp_samples.append(curve)
                assert opcode == 2 and len(body) > 13 and body[0] == 1
                subscription = struct.unpack("<I", body[1:5])[0]
                timestamp = struct.unpack("<Q", body[5:13])[0]
                payload = json.loads(body[13:])
                if subscription == 41:
                    assert payload["synthetic"] and payload.get("state_dim", 11) == 11
                    assert payload["timestamp_ns"] == timestamp
                    validate_control_sample(payload, samples[-1] if samples else None)
                    validate_angle_sample(payload)
                    if samples and payload["target"].get("state_valid") and samples[-1]["target"].get("state_valid") \
                            and payload["update_count"] >= samples[-1]["update_count"]:
                        a, b = samples[-1]["target"], payload["target"]
                        delta = math.remainder(b["phase"] - a["phase"], 2 * math.pi)
                        assert abs((b["phase_unwrapped"] - a["phase_unwrapped"]) - delta) < 1e-6, \
                            "phase curve jumped at yaw wrap"
                    samples.append(payload)
                elif subscription == 42:
                    assert payload["format"] == "jpeg"
                    assert jpeg_dimensions(base64.b64decode(payload["data"], validate=True)) == (1440, 1080)
                    images += 1
                elif subscription == 43:
                    entities = payload["entities"]
                    assert {entity["id"] for entity in entities} == {"measured", "predicted", "center"}
                    assert all(entity["frame_id"] == "world" for entity in entities)
                    scenes += 1
                else:
                    raise AssertionError(f"wrong subscription id {subscription}")
            result = {
                "metrics": len(samples), "jpeg_images": images, "scenes": scenes, "udp_curves": len(udp_samples),
                "tracking_samples": sum(bool(s["tracking"]) for s in samples),
                "converged_samples": sum(bool(s.get("converged")) for s in samples),
                "max_processing_ms": max((s["processing_ms"] for s in samples), default=0),
                "max_frame_age_ms": max((s["frame_age_ms"] for s in samples), default=0),
                "valid_plan_samples": sum(bool(s["planned_control"]) for s in samples),
                "fire_samples": sum(bool(s["fire"]) for s in samples),
                "shots_requested": samples[-1]["shots_requested"] if samples else 0,
                "launch_count": samples[-1]["launch_count"] if samples else 0,
                "hit_count": samples[-1]["hit_count"] if samples else 0,
                "fire_reasons": {reason: sum(s["fire_reason"] == reason for s in samples)
                                 for reason in sorted({s["fire_reason"] for s in samples})},
            }
            for name, begin, end in [("spin", 2, 3.8), ("stopped", 5, 5.9), ("reverse", 8, 9)]:
                window = [s for s in samples if begin <= s["frame_seq"] / 100 < end and s["tracking"]]
                assert window, f"no tracking sample during {name}"
                assert any(s["planned_control"] and s["solver_ok"] for s in window), \
                    f"no usable Planner output during {name}"
                sample = window[-1]
                result[name] = {key: sample.get(key) for key in [
                    "frame_seq", "state_dim", "converged", "center_x", "center_y", "vyaw",
                    "radius", "dh1", "dh2", "nis", "accepted"]}
            print(json.dumps(result, indent=2))
            assert len(samples) >= 60 and images >= 60 and scenes >= 60, "missing Foxglove streams"
            assert len(udp_samples) >= 100, "PlotJuggler UDP curves missing"
            assert sum(c["plan"]["valid"] for c in udp_samples) > 0, "UDP received no plan curves"
            assert result["tracking_samples"] >= len(samples) * 0.9, "synthetic tracking dropouts"
            assert result["spin"]["vyaw"] > 1.5, "positive rotation did not converge"
            assert abs(result["stopped"]["vyaw"]) < 0.5, "stop did not settle"
            assert result["reverse"]["vyaw"] < -1.5, "reversed rotation did not converge"
            assert result["valid_plan_samples"] > 0, "Planner never produced a usable command"
            # Foxglove samples at 10 Hz while Shooter evaluates at 100 Hz, so the
            # cumulative counter is authoritative even if individual pulses fall
            # between debug samples.
            assert result["shots_requested"] > 0, "closed loop never requested a shot"
            assert result["launch_count"] == result["shots_requested"], \
                "synthetic actuator lost a launch request"
            assert result["hit_count"] == 0, "synthetic feed must not fabricate hit results"
        finally:
            failed = sys.exc_info()[0] is not None
            udp.close()
            if client:
                client.socket.close()
            if failed and process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=2)
            log.seek(0)
            output = log.read()
            if failed or process.returncode:
                print(output[-4000:], file=sys.stderr)
            if process.returncode and not failed:
                raise RuntimeError(f"task_sim exited with {process.returncode}")
    print("task_sim + Foxglove synthetic integration passed "
          "(Planner / Shooter launches / rotation / stop / reversal)")


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else "./build/task_sim")
