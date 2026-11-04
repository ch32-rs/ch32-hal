#!/usr/bin/env python3
"""Raw Ethernet throughput peer for the `eth_raw_bench` firmware.

Talks to the board at the Ethernet layer through a macOS BPF socket, so it
needs neither an IPv4 address nor any routing setup on the host interface:

    sudo python3 eth_raw_bench.py en9

The board runs the `eth_raw_bench` example (MAC 02:00:00:00:00:48, EtherType
0x88b5). Downlink asks the board to transmit a burst of 1514-byte frames;
uplink blasts the same size frames at the board, which counts them and
acknowledges. Reported rates are Ethernet frame bits per second, including
headers, so the 100 Mbit/s line rate is the ceiling.
"""

import argparse
import fcntl
import os
import re
import statistics
import struct
import subprocess
import time

BIOCGBLEN = 0x40044266
BIOCSBLEN = 0xC0044266
BIOCSETIF = 0x8020426C
BIOCSRTIMEOUT = 0x8010426D
BIOCIMMEDIATE = 0x80044270
BIOCSHDRCMPLT = 0x80044275
BIOCSSEESENT = 0x80044277

BOARD_MAC = bytes.fromhex("020000000048")
ETHERTYPE = b"\x88\xb5"
HEADER = 14
FRAME_SIZE = 1514
CONTROL_SIZE = 60
BUFFER_SIZE = 1 << 20
READ_TIMEOUT = 0.05

CMD_DATA = ord("F")
CMD_DOWN = ord("D")
CMD_UP = ord("U")
CMD_PING = ord("P")
CMD_ACK = ord("A")


def interface_mac(ifname):
    text = subprocess.check_output(["ifconfig", ifname], text=True)
    match = re.search(r"\bether\s+([0-9a-f:]{17})", text)
    if not match:
        raise RuntimeError(f"no ether address on {ifname}")
    return bytes.fromhex(match.group(1).replace(":", ""))


def open_bpf(ifname):
    fd = None
    for index in range(256):
        try:
            fd = os.open(f"/dev/bpf{index}", os.O_RDWR)
            break
        except FileNotFoundError:
            raise RuntimeError("no /dev/bpf device available") from None
        except PermissionError:
            raise RuntimeError("permission denied: run with sudo") from None
        except OSError:
            continue
    if fd is None:
        raise RuntimeError("all BPF devices are busy")

    fcntl.ioctl(fd, BIOCSBLEN, struct.pack("<I", BUFFER_SIZE))
    request = ifname.encode()[:15].ljust(16, b"\0") + b"\0" * 16
    fcntl.ioctl(fd, BIOCSETIF, request)
    for request_id, value in (
        (BIOCIMMEDIATE, 1),
        (BIOCSHDRCMPLT, 1),
        (BIOCSSEESENT, 0),
    ):
        fcntl.ioctl(fd, request_id, struct.pack("<I", value))
    seconds = int(READ_TIMEOUT)
    useconds = int((READ_TIMEOUT - seconds) * 1_000_000)
    fcntl.ioctl(fd, BIOCSRTIMEOUT, struct.pack("<qii", seconds, useconds, 0))
    return fd, struct.unpack("<I", fcntl.ioctl(fd, BIOCGBLEN, b"\0" * 4))[0]


class Frames:
    """Reads the board's frames from the BPF device, buffering whole reads."""

    def __init__(self, fd, buffer_length, host, board):
        self.fd = fd
        self.buffer_length = buffer_length
        self.host = host
        self.board = board
        self.pending = []

    def _read(self):
        try:
            data = os.read(self.fd, self.buffer_length)
        except OSError as error:
            if getattr(error, "errno", None) == 35:  # EAGAIN
                return
            raise
        offset = 0
        while offset + 20 <= len(data):
            header_length = struct.unpack_from("<H", data, offset + 16)[0]
            captured = struct.unpack_from("<I", data, offset + 8)[0]
            start = offset + header_length
            frame = data[start : start + captured]
            if (
                frame[:6] == self.host
                and frame[6:12] == self.board
                and frame[12:14] == ETHERTYPE
            ):
                self.pending.append(frame)
            offset = (start + captured + 3) & ~3

    def next(self, deadline):
        while True:
            if self.pending:
                return self.pending.pop(0)
            if time.monotonic() >= deadline:
                return None
            self._read()


def send(fd, frame):
    if os.write(fd, frame) != len(frame):
        raise RuntimeError("short BPF frame write")


def control(host, command, count):
    frame = bytearray(CONTROL_SIZE)
    frame[:6] = BOARD_MAC
    frame[6:12] = host
    frame[12:14] = ETHERTYPE
    frame[14] = command
    frame[15:19] = count.to_bytes(4, "little")
    return bytes(frame)


def data_frame(host):
    frame = bytearray(FRAME_SIZE)
    frame[:6] = BOARD_MAC
    frame[6:12] = host
    frame[12:14] = ETHERTYPE
    frame[14] = CMD_DATA
    pattern = bytes(range(255, 0, -1))
    remaining = FRAME_SIZE - 15
    frame[15:] = (pattern * (remaining // len(pattern) + 1))[:remaining]
    return bytes(frame)


def wait_ack(frames, deadline, count=None):
    while True:
        frame = frames.next(deadline)
        if frame is None:
            return None
        if len(frame) >= HEADER + 5 and frame[14] == CMD_ACK:
            value = int.from_bytes(frame[15:19], "little")
            if count is None or value == count:
                return value


def measure_downlink(fd, frames, host, count, timeout):
    send(fd, control(host, CMD_DOWN, count))
    start = time.monotonic()
    deadline = start + timeout
    received = 0
    while True:
        frame = frames.next(deadline)
        if frame is None:
            raise TimeoutError(f"downlink stalled after {received}/{count} frames")
        if frame[14] == CMD_DATA:
            received += 1
        elif frame[14] == CMD_ACK:
            elapsed = time.monotonic() - start
            acknowledged = int.from_bytes(frame[15:19], "little")
            return elapsed, received, acknowledged


def measure_uplink(fd, frames, host, count, payload, timeout, rate, batch):
    # Writing every frame as fast as Python can fills the USB NIC's transmit
    # queue and silently drops the excess, so pace the writes to wire time.
    send(fd, control(host, CMD_UP, count))
    deadline = time.monotonic() + timeout
    start = time.monotonic()
    seconds_per_frame = FRAME_SIZE * 8 / rate
    sent = 0
    while sent < count:
        upper = min(count, sent + batch)
        for _ in range(sent, upper):
            send(fd, payload)
        sent = upper
        delay = start + sent * seconds_per_frame - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    value = wait_ack(frames, deadline)
    elapsed = time.monotonic() - start
    if value is None:
        raise TimeoutError("uplink acknowledgement lost")
    return elapsed, value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--frames", type=int, default=8192)
    parser.add_argument("--direction", choices=("both", "up", "down"), default="both")
    parser.add_argument("--timeout", type=float, default=6.0)
    parser.add_argument("--rate", type=float, default=90e6, help="uplink pacing in bit/s")
    parser.add_argument("--batch", type=int, default=64, help="frames per write burst")
    args = parser.parse_args()
    if args.rounds <= 0 or args.frames <= 0 or args.timeout <= 0:
        parser.error("rounds, frames and timeout must be positive")

    host = interface_mac(args.interface)
    fd, buffer_length = open_bpf(args.interface)
    frames = Frames(fd, buffer_length, host, BOARD_MAC)
    payload = data_frame(host)
    directions = ("up", "down") if args.direction == "both" else (args.direction,)
    results = {direction: [] for direction in directions}

    print(
        f"{args.interface}: host {host.hex(':')} -> board {BOARD_MAC.hex(':')}, "
        f"{args.frames} x {FRAME_SIZE} byte frames"
    )
    try:
        send(fd, control(host, CMD_PING, 0))
        if wait_ack(frames, time.monotonic() + args.timeout, 0) is None:
            raise TimeoutError("board did not answer a ping; is eth_raw_bench running?")
        for round_index in range(args.rounds):
            for direction in directions:
                if direction == "down":
                    elapsed, received, acknowledged = measure_downlink(
                        fd, frames, host, args.frames, args.timeout
                    )
                    bits = received * FRAME_SIZE * 8
                    loss = args.frames - received
                    print(
                        f"round {round_index + 1} down: {bits / elapsed / 1e6:8.2f} Mbit/s "
                        f"({received}/{args.frames} frames, {loss} missing)"
                    )
                else:
                    elapsed, acknowledged = measure_uplink(
                        fd, frames, host, args.frames, payload, args.timeout, args.rate, args.batch
                    )
                    bits = acknowledged * FRAME_SIZE * 8
                    print(
                        f"round {round_index + 1} up:   {bits / elapsed / 1e6:8.2f} Mbit/s "
                        f"({acknowledged}/{args.frames} frames)"
                    )
                results[direction].append(bits / elapsed / 1e6)
    finally:
        os.close(fd)

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
