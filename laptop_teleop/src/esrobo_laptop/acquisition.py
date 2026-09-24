"""Laptop entrypoints for the existing, calibrated PICO and SenseGlove bridges."""
import importlib.util
import json
import socket
import sys
import time

from .config import WORKSPACE


def load_bridge(name):
    path = WORKSPACE / "teleoperation" / "bridges" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"_laptop_{name}", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def annotate(packet, kind, stamps, physical_sides):
    packet = dict(packet)
    packet["laptop_input"] = dict(v=1, kind=kind, sample_monotonic=stamps,
                                  physical_sides=list(physical_sides))
    return json.dumps(packet, separators=(",", ":"), allow_nan=False).encode()


class AnnotatedSocket:
    def __init__(self, sock, metadata):
        self.sock, self.metadata = sock, metadata

    def sendto(self, encoded, destination):
        kind, stamps, sides = self.metadata()
        return self.sock.sendto(annotate(json.loads(encoded), kind, stamps, sides), destination)


def pico(argv):
    from .pico_hand import PicoHands
    bridge = load_bridge("xrobotoolkit_body_udp_bridge")
    original = bridge._build_packet
    args = bridge.parse_args(argv)
    hands = PicoHands()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_print = 0.

    def build(sdk, *positional, **kwargs):
        nonlocal last_print
        samples = hands.poll(sdk)
        stamp = time.monotonic()
        if samples and not args.record_only:
            packet = dict(pico_hands=samples)
            sock.sendto(annotate(packet, "pico_hand", {s: stamp-v["age_s"] for s, v in samples.items()}, samples),
                        (args.host, args.port))
        if stamp - last_print > 2:
            print(f"[pico_hand] {hands.status}", flush=True)
            last_print = stamp
        packet, count = original(sdk, *positional, **kwargs)
        if packet is not None:
            packet["pico_hand_status"] = dict(hands.status)
            packet = json.loads(annotate(packet, "pico", dict(left=stamp, right=stamp), ("left", "right")))
        return packet, count

    bridge._build_packet = build
    try:
        return bridge.main(argv)
    finally:
        sock.close()


def senseglove(argv):
    bridge = load_bridge("senseglove_ros_to_esrobo_hand_bridge")
    original = bridge.SenseGloveUdpBridge.send

    def send(self, sock, calibration):
        def metadata():
            sources = {"left": self.left, "right": self.right}
            if self.args.swap_left_right_targets:
                sources = {"left": self.right, "right": self.left}
            sides = (self.args.single_side,) if self.args.single_side else ("left", "right")
            if self.args.single_side and self.args.swap_left_right_targets:
                sides = ("right" if self.args.single_side == "left" else "left",)
            stamps = {
                s: (v.stamp_monotonic if v.has_live_sensor_payload()
                    and v.imu_quat_wxyz is not None
                    and (v.packets_per_second_received is None or v.packets_per_second_received > 0)
                    else 0.)
                for s, v in sources.items()
            }
            return "senseglove", stamps, sides
        return original(self, AnnotatedSocket(sock, metadata), calibration)

    bridge.SenseGloveUdpBridge.send = send
    return bridge.main(argv)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] not in ("pico", "senseglove"):
        print("usage: python -m esrobo_laptop.acquisition {pico|senseglove} [bridge arguments]")
        return 2
    return (pico if argv[0] == "pico" else senseglove)(argv[1:])


if __name__ == "__main__":
    raise SystemExit(main())
