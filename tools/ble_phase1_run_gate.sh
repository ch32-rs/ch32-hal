#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${BIN:-$ROOT/examples/ch32v208/target/riscv32imc-unknown-none-elf/release/ble_peripheral_phase1_adv}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="${LOG:-/tmp/task68_phase1_gate_${STAMP}_sdi.log}"
PYLOG="${PYLOG:-/tmp/task68_phase1_gate_${STAMP}_mac.log}"
NAME="${NAME:-Simple}"
SCAN_SECONDS="${SCAN_SECONDS:-8}"
CONNECT_AT="${CONNECT_AT:-0}"

if [[ ! -x "$BIN" && ! -f "$BIN" ]]; then
  echo "missing binary: $BIN" >&2
  exit 2
fi

echo "BIN=$BIN"
shasum -a 256 "$BIN"
echo "SDI_LOG=$LOG"
echo "MAC_LOG=$PYLOG"

timeout 60s wlink -q flash --enable-sdi-print --watch-serial "$BIN" >"$LOG" 2>&1 &
WPID=$!

cleanup() {
  kill "$WPID" 2>/dev/null || true
  wait "$WPID" 2>/dev/null || true
}
trap cleanup EXIT

# Let firmware finish PHY init and start advertising before CoreBluetooth scans.
sleep 6

set +e
python3 "$ROOT/tools/ble_phase1_verify.py" \
  --name "$NAME" \
  --scan-seconds "$SCAN_SECONDS" \
  --connect \
  --connect-at "$CONNECT_AT" \
  --connect-timeout 8 \
  --hold-seconds 1 \
  --sdi-log "$LOG" \
  --require-connect-ind | tee "$PYLOG"
RC=${PIPESTATUS[0]}
set -e

sleep 1
cleanup
trap - EXIT

echo "RESULT=$RC"
grep -E "CONNECT_IND|RX_TURNAROUND|scheduler|tx#" "$LOG" | tail -80 || true
exit "$RC"
