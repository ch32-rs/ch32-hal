#!/usr/bin/env python3
"""Linux AF_PACKET peer for hal_eth; requires CAP_NET_RAW or root."""

import argparse
import socket
import time

MAGIC = b"CH32V4-ECHO"
ETHERTYPE = b"\x88\xb5"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    parser.add_argument("--mac", default="02:00:00:00:00:47")
    parser.add_argument("--rounds", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()
    destination = bytes.fromhex(args.mac.replace(":", ""))
    if len(destination) != 6 or args.rounds <= 0 or args.timeout <= 0:
        parser.error("MAC must have six bytes; rounds and timeout must be positive")
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x88B5)) as peer:
        peer.bind((args.interface, 0))
        source = peer.getsockname()[4]
        count = 0
        total = 0
        for sequence in range(args.rounds):
            for length in [46, 47, 128, 512, 1500]:
                prefix = MAGIC + sequence.to_bytes(4, "big") + length.to_bytes(2, "big")
                payload = prefix + bytes((sequence + offset) & 255 for offset in range(length - len(prefix)))
                frame = destination + source + ETHERTYPE + payload
                if peer.send(frame) != len(frame):
                    raise RuntimeError("short raw Ethernet send")
                deadline = time.monotonic() + args.timeout
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f"no echo for sequence={sequence} payload={length}")
                    peer.settimeout(remaining)
                    echoed = peer.recv(2048)
                    if echoed[:14] != source + destination + ETHERTYPE:
                        continue
                    if echoed[14:] != payload:
                        raise RuntimeError(f"echo mismatch for sequence={sequence} payload={length}")
                    break
                count += 1
                total += len(payload)
        print(f"PASS: {count} frames, {total} payload bytes; interface={args.interface}")


if __name__ == "__main__":
    main()
