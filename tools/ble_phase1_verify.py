#!/usr/bin/env python3
"""macOS BLE validator for CH32 BLE PeripheralRole phase-1 probes.

The script uses Bleak/CoreBluetooth to:
  1. scan for a target local name,
  2. optionally initiate a central connection,
  3. optionally check an SDI log for the firmware CONNECT_IND marker.

On macOS the device address reported by CoreBluetooth is a UUID-like identifier.
That is fine: connecting by the discovered BLEDevice still causes the central to
send CONNECT_IND over the air.
"""

from __future__ import annotations

import argparse
import asyncio
import pathlib
import sys
import time

from bleak import BleakClient, BleakScanner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default="Simple", help="substring of advertised local name")
    parser.add_argument("--scan-seconds", type=float, default=12.0)
    parser.add_argument("--connect", action="store_true", help="connect to the first matching device")
    parser.add_argument("--connect-timeout", type=float, default=8.0)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--sdi-log", type=pathlib.Path, help="firmware SDI log to check after connect")
    parser.add_argument("--require-connect-ind", action="store_true")
    return parser.parse_args()


async def scan(name: str, seconds: float) -> tuple[list[object], int]:
    hits = 0
    devices = {}

    def on_adv(device, advertisement_data):
        nonlocal hits
        local_name = advertisement_data.local_name or device.name or ""
        if name in local_name:
            hits += 1
            devices[device.address] = device
            print(f"HIT #{hits} name={local_name} addr={device.address} rssi={advertisement_data.rssi}")

    scanner = BleakScanner(on_adv)
    await scanner.start()
    await asyncio.sleep(seconds)
    await scanner.stop()
    return list(devices.values()), hits


async def connect_once(device, timeout: float, hold: float) -> bool:
    print(f"CONNECT addr={device.address} name={device.name}")
    try:
        async with BleakClient(device, timeout=timeout) as client:
            ok = client.is_connected
            print(f"CONNECTED {ok}")
            if ok and hold > 0:
                await asyncio.sleep(hold)
            return ok
    except Exception as exc:  # noqa: BLE001 - diagnostic tool should print backend errors
        print(f"CONNECT_ERROR {type(exc).__name__}: {exc}")
        return False


def check_sdi_log(path: pathlib.Path) -> bool:
    # Give the wlink watch process time to flush after the host connect attempt.
    time.sleep(0.5)
    text = path.read_text(errors="replace") if path.exists() else ""
    marker = "# CONNECT_IND" in text and "adv_match=true" in text
    print(f"SDI_CONNECT_IND {marker} path={path}")
    if marker:
        for line in text.splitlines():
            if "# CONNECT_IND" in line:
                print(line)
    return marker


async def main() -> int:
    args = parse_args()
    devices, hits = await scan(args.name, args.scan_seconds)
    print(f"DONE scan hits={hits} devices={len(devices)}")
    if hits == 0:
        return 2

    connected = True
    if args.connect:
        connected = await connect_once(devices[0], args.connect_timeout, args.hold_seconds)

    marker = True
    if args.sdi_log:
        marker = check_sdi_log(args.sdi_log)

    if args.require_connect_ind and not marker:
        return 4
    if args.connect and not connected:
        # Keep this distinct from RF invisibility; CoreBluetooth connection can
        # fail after CONNECT_IND due missing LL connection state machine.
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
