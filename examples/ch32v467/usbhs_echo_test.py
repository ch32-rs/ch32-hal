"""Check the hal_usbhs CDC echo on a Linux ttyACM device."""

import argparse
import fcntl
import os
import select
import struct
import termios
import time
import tty


def exchange(port, payload, timeout):
    deadline = time.monotonic() + timeout
    sent = 0
    received = bytearray()
    while sent != len(payload) or len(received) < len(payload):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"sent {sent}/{len(payload)}, received {len(received)} bytes")
        readable, writable, _ = select.select([port], [port] if sent < len(payload) else [], [], remaining)
        if writable:
            sent += os.write(port, payload[sent:])
        if readable:
            chunk = os.read(port, 8192)
            if not chunk:
                raise ConnectionError("CDC device disconnected")
            received.extend(chunk)
    if received != payload:
        raise AssertionError(f"echo mismatch for {len(payload)} bytes")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="CDC device, for example /dev/ttyACM0")
    parser.add_argument("--rounds", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    if args.rounds <= 0 or args.timeout <= 0:
        parser.error("rounds and timeout must be positive")
    port = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    previous = None
    try:
        previous = termios.tcgetattr(port)
        tty.setraw(port, termios.TCSANOW)
        attributes = termios.tcgetattr(port)
        attributes[2] |= termios.CLOCAL | termios.CREAD
        attributes[4] = termios.B115200
        attributes[5] = termios.B115200
        termios.tcsetattr(port, termios.TCSANOW, attributes)
        termios.tcflush(port, termios.TCIOFLUSH)
        fcntl.ioctl(port, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
        total = 0
        for round_index in range(args.rounds):
            for length in [1, 7, 63, 64, 65, 127, 128, 129, 511, 512, 513, 4096]:
                payload = bytes((offset * 37 + round_index * 17) & 255 for offset in range(length))
                exchange(port, payload, args.timeout)
                total += length
        print(f"PASS: {args.rounds * 12} exchanges, {total} bytes matched")
    finally:
        try:
            if previous is not None:
                termios.tcsetattr(port, termios.TCSANOW, previous)
        finally:
            os.close(port)


if __name__ == "__main__":
    main()
