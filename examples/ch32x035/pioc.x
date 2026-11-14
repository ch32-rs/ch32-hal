/* PIOC owns the top 4 KiB of SRAM as its program window: the eMCU executes
   from 0x2000_4000 and payloads are written there by the host. `memory-x`
   hands the whole 20 KiB to `RAM`, so the stack has to start below it. */
_stack_start = ORIGIN(RAM) + LENGTH(RAM) - 0x1000;
