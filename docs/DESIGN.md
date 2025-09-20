# Design / Architecture (Microwatt + FIR peripheral)

## Overview
This submission demonstrates a Microwatt-friendly accelerator pattern: a small, parameterized FIR accelerator implemented in SystemVerilog, with a Wishbone-lite register wrapper for memory-mapped access.

## Blocks
- `fir_accel` — FIR streaming accelerator (verilog/rtl/accel/fir_accel.sv)
- `wishbone_fir_if` — Wishbone-lite wrapper to map CPU accesses to control, data, and coefficient writes (verilog/rtl/bus/wishbone_fir_if.sv)
- `tb_fir_top` — simulation-only testbench (verilog/dv/tb_fir_top.sv)

## Memory Map (base = 0x4000_0000)
- 0x4000_0000 + 0x00 : CTRL  (write: bit0 = START)
- 0x4000_0000 + 0x04 : DATAIN (write input sample)
- 0x4000_0000 + 0x08 : DATAOUT (read result)
- 0x4000_0000 + 0x0C : STATUS (bit0 = DONE)
- 0x4000_0000 + 0x10.. : COEFF[0..N-1] (write coefficients)

## Simulation
- `verilog/dv/tb_fir_top.sv` provides a self-contained simulation showing:
  - coeff loads (via coeff_wr interface)
  - feeding samples and printing results
  - VCD waveform produced (`fir_tb.vcd`)

## Next steps (integration plan)
1. Instantiate `wishbone_fir_if` in the OpenFrame wrapper/top alongside Microwatt.
2. Connect Microwatt's bus to the wishbone wrapper (address decode at base 0x4000_0000).
3. Add an example firmware binary that uses the memory map (sw/examples/accel_test.c).
4. Verify end-to-end in QEMU/Sim or on FPGA using the Microwatt environment.

License: MIT (or Apache-2.0)
