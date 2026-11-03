# CH32V4 I3C host tests

Run the production controller and transfer state machines against the generated
V4x7 register API with two control entries and eight-byte data FIFO models:

```sh
CARGO_TARGET_DIR=/tmp/ch32v4-i3c-host-target cargo test \
  --manifest-path tools/i3c-host-tests/Cargo.toml \
  --target x86_64-unknown-linux-gnu -- --test-threads=1
```

`--target x86_64-unknown-linux-gnu` is the Linux target of the reference
command; no CI job currently runs this harness. It is a plain `std` crate, so
omit `--target` to run natively on macOS (all five pre-existing harnesses
passed this way on 2026-11-03).

The harness compiles the production V4 DMA backend and I3C driver. It replaces
GPIO/RCC/interrupt plumbing and register side effects. Tests cover merged
same-direction I2C operations, private/CCC repeated starts, FIFO boundaries,
frame completion, private transfers, CCC prefixes, ENTDAA
identities/capacity/address validation, short reads, errors, cancellation,
timeouts, clock changes and driver lifecycle. Legacy I2C direction changes are
rejected before bus activity, and the model asserts the RM requirement that
MTYPE=4 always uses MEND=1.

DMA tests use DMA1 CH4 TX / CH5 RX with byte widths and move actual buffer bytes
through the modeled eight-byte FIFOs. They cover 1/3/4/7/8/9/31/257/65535-byte
transfers, both private repeated-start direction orders, legacy transactions,
scatter/CCC FIFO fallback, DMA completion before STOP, STOP before DMA
completion, residual counts, checked DMA error IRQ wakeups, simultaneous
peripheral errors, short reads, cancellation, timeouts and explicit recovery. No
test performs DMA against the host's truncated 32-bit pointer directly; buffers
are registered with the model to resolve the programmed address to the original
host allocation.

The DMA backend deliberately retains FIFO servicing for more than one buffer per
direction and for CCC/ENTDAA. Control DMA is not used. This avoids assuming a
bounded buffer-reload latency between messages or scatter segments.

The model advances on register access. It does not prove interrupt latency, FIFO
service at any bus speed, electrical timing, address parity on the wire, target
interoperability, bus recovery or the hardware's interpretation of raw timings.
ENTDAA no-target completion is tested as one possible hardware outcome; an
unacknowledged initial broadcast reported as ERRF remains an error.
