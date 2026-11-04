#!/usr/bin/env python3
"""TCP throughput peer for hal_eth_bench.

The board runs a TCP server on 192.168.47.1:5001. Give the wired host
interface an address in the same subnet first, for example:

    sudo ifconfig en9 inet 192.168.47.2 netmask 255.255.255.0
    python3 eth_throughput_test.py 192.168.47.1

Uplink streams payload to the board (the board counts and discards it).
Downlink has the board generate the payload. Both directions print Mbit/s
from the host's monotonic clock.
"""

import argparse
import socket
import statistics
import time

BLOCK = bytes(range(256)) * 256  # 64 KiB


def read_exactly(sock, count):
    view = bytearray(min(count, 65536))
    remaining = count
    while remaining:
        want = min(remaining, len(view))
        received = sock.recv_into(memoryview(view)[:want])
        if received == 0:
            raise ConnectionError(f"connection closed with {remaining} bytes left")
        remaining -= received


def read_line(sock):
    line = bytearray()
    while True:
        byte = sock.recv(1)
        if not byte:
            raise ConnectionError("connection closed while reading a line")
        if byte == b"\n":
            return bytes(line)
        line += byte


def send_payload(sock, count):
    view = memoryview(BLOCK)
    remaining = count
    while remaining:
        chunk = min(remaining, len(view))
        sock.sendall(view[:chunk])
        remaining -= chunk


def measure(sock, direction, count):
    if direction == "up":
        start = time.monotonic()
        sock.sendall(f"UP {count}\n".encode())
        send_payload(sock, count)
        reply = read_line(sock)
        elapsed = time.monotonic() - start
    else:
        start = time.monotonic()
        sock.sendall(f"DOWN {count}\n".encode())
        read_exactly(sock, count)
        elapsed = time.monotonic() - start
        reply = read_line(sock)
    if reply != f"OK {count}".encode():
        raise RuntimeError(f"unexpected reply for {direction}: {reply!r}")
    return count * 8 / elapsed / 1e6


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", nargs="?", default="192.168.47.1")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--bytes", type=int, default=8 * 1024 * 1024)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--direction", choices=("both", "up", "down"), default="both")
    args = parser.parse_args()
    if args.bytes <= 0 or args.rounds <= 0:
        parser.error("bytes and rounds must be positive")

    directions = ("up", "down") if args.direction == "both" else (args.direction,)
    results = {direction: [] for direction in directions}

    with socket.create_connection((args.host, args.port), timeout=10) as sock:
        sock.settimeout(30)
        print(
            f"connected {sock.getsockname()[0]} -> {args.host}:{args.port}, "
            f"{args.bytes} bytes x {args.rounds} rounds"
        )
        for round_index in range(args.rounds):
            for direction in directions:
                mbits = measure(sock, direction, args.bytes)
                results[direction].append(mbits)
                print(f"round {round_index + 1} {direction}: {mbits:8.2f} Mbit/s")
        sock.sendall(b"QUIT\n")

    print()
    for direction in directions:
        samples = results[direction]
        print(
            f"{direction}: median {statistics.median(samples):8.2f} Mbit/s, "
            f"best {max(samples):8.2f} Mbit/s"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
