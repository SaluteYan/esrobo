"""Laptop read-only smoke test; never sends a joint target."""
import argparse
import json
import time

from .client import RobotClient


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", required=True)
    ap.add_argument("--port", type=int, default=16000)
    ap.add_argument("--key-file", required=True)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    client = RobotClient(args.host, args.port, args.key_file)
    try:
        state = client.connect()
        print(json.dumps(state["contract"], ensure_ascii=False, indent=2))
        end, next_print = time.monotonic() + args.seconds, 0.0
        while time.monotonic() < end:
            state = client.receive()
            if time.monotonic() >= next_print:
                print(json.dumps(state, ensure_ascii=False), flush=True)
                next_print = time.monotonic() + 1
    finally:
        # Read-only monitor must not request a stop; close just its socket.
        client.socket.close()


if __name__ == "__main__":
    main()
