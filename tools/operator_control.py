"""Local simulator authority client. Commands expire unless refreshed explicitly."""
import argparse
import json
import socket
import time

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--aim", action="store_true")
parser.add_argument("--authorize", action="store_true")
parser.add_argument("--spin", action="store_true")
parser.add_argument("--seconds", type=float, default=0, help="0: query status only")
args = parser.parse_args()
if args.authorize and not args.aim:
    parser.error("--authorize requires --aim")
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.settimeout(1)
    sock.connect(("127.0.0.1", 5001))
    sock.send(b"status")
    state = json.loads(sock.recv(4096))
    print(json.dumps(state, ensure_ascii=False))
    session = state["session"]
    seq = state["applied_seq"]
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            seq += 1
            command = dict(session=session, seq=seq, sent_ms=time.time_ns() // 1_000_000,
                           ttl_ms=300, aim_enabled=args.aim,
                           fire_authorized=args.authorize, spin_enabled=args.spin)
            sock.send(json.dumps(command).encode())
            state = json.loads(sock.recv(4096))
            if state["session"] != session or state["reason"] != "applied":
                raise RuntimeError(f"Command rejected; re-arm explicitly: {state}")
            time.sleep(0.05)
    finally:
        if args.seconds > 0:
            sock.send(json.dumps(dict(session=session, seq=seq + 1,
                sent_ms=time.time_ns() // 1_000_000, ttl_ms=300,
                aim_enabled=False, fire_authorized=False, spin_enabled=False)).encode())