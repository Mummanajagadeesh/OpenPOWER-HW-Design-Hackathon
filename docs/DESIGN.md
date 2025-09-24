# FIR Accelerator — Design Documentation

---

## Overview

This document provides a **technical reference for the FIR accelerator**, including:

* Memory map for CPU access (Microwatt/OpenFrame).
* Register-level descriptions.
* Data path architecture.
* Timing and latency considerations.
* Integration instructions for SoC (OpenFrame) and ASIC/FPGA flows.

The accelerator consists of:

* **`fir_core`** — sequential MAC FIR datapath (shift register + coefficient memory + accumulator).
* **`fir_wb`** — Wishbone-Lite memory-mapped wrapper for CPU interface.
* Optional **FIFO (`fifo_sync`)** for streaming input.

---

## Memory Map

| Address Offset | Name      | Width | Type | Description                                                                                              |
| -------------- | --------- | ----- | ---- | -------------------------------------------------------------------------------------------------------- |
| `0x00`         | CTRL      | 32    | RW   | Control register. Bit0 = START. CPU sets to 1 to trigger FIR computation. Must clear to 0 after readout. |
| `0x04`         | STATUS    | 32    | RO   | Status register. Bit1 = OUT\_VALID. Bit0 reserved. Indicates FIR output is valid.                        |
| `0x08`         | DATAOUT   | 32    | RO   | Output of FIR computation (`sample_out`).                                                                |
| `0x80`         | IN\_FIFO  | 32    | WO   | Input FIFO register. CPU writes new sample to trigger computation (if FIFO enabled).                     |
| `0x10 + 4*i`   | COEFF\[i] | 32    | WO   | Coefficient registers. Supports up to `MAX_TAPS` coefficients. Writing pulses `coeff_wr_en` to FIR core. |

**Notes:**

* All registers are **32-bit aligned**.
* Writes to COEFF are **single-cycle pulses**; FIR core latches data internally.
* CPU must manage **START semantics**: write `1` to CTRL.START, poll STATUS.OUT\_VALID, then write `0` to CTRL.START.

---

## Register Details

### CTRL (0x00)

| Bit  | Name     | Type | Description                                                            |
| ---- | -------- | ---- | ---------------------------------------------------------------------- |
| 0    | START    | RW   | Set to 1 to start FIR. Level-sensitive: holds DONE high until cleared. |
| 31:1 | RESERVED | RW   | Reserved for future use                                                |

### STATUS (0x04)

| Bit | Name       | Type | Description                    |
| --- | ---------- | ---- | ------------------------------ |
| 1   | OUT\_VALID | RO   | High when FIR output is ready. |
| 0   | RESERVED   | RO   | Reserved                       |

### DATAOUT (0x08)

* Read-only: contains **FIR core output sample**.
* Width: 32 bits (signed).

### IN\_FIFO (0x80)

* Write-only: input sample for FIR.
* Optional FIFO can decouple CPU timing from FIR computation.

### COEFF\[i] (0x10 + 4\*i)

* Write-only coefficient register.
* Pulses `coeff_wr_en` to FIR core on write.
* CPU sequence:

```c
for (int i=0; i<TAPS; i++) {
    write32(FIR_BASE + 0x10 + 4*i, coeff[i]);
}
```

---

## Data Path Architecture

```
          ┌─────────────┐
  sample →│ IN FIFO?    │
          └─────────────┘
                 │
                 ▼
          ┌─────────────┐
          │ Shift Regs  │ <── Coeff Memory
          │ [0..TAPS-1] │
          └─────────────┘
                 │
                 ▼
          ┌─────────────┐
          │  MAC Loop   │
          │ acc += x*h  │
          └─────────────┘
                 │
                 ▼
          ┌─────────────┐
          │  Saturation │
          └─────────────┘
                 │
                 ▼
            sample_out
```

* Sequential multiply-accumulate (MAC) over `TAPS` cycles.
* Accumulator width = `ACC_W` (typically `2*DATA_W`) to prevent overflow.
* Output is saturated/truncated to `DATA_W`.
* `DONE` signal asserted when output is valid.

---

## Timing & Latency

* **Latency**: `TAPS` cycles (MAC) + FSM overhead (\~1–2 cycles).

* **Throughput**: one output every `TAPS` cycles (sequential MAC).

* **Control**:

  * START level-sensitive.
  * DONE stays high until START cleared.

* CPU must poll STATUS.OUT\_VALID before reading DATAOUT.

* Optional FIFO allows **decoupled write rate**:

```
CPU writes → IN_FIFO → FIR core
```

---

## Integration with Microwatt/OpenFrame

1. **Instantiate `fir_wb`** at base address (example: `0x4000_0000`).
2. Connect Wishbone-Lite signals from CPU bus.
3. Optional: add FIFO to decouple CPU clock from FIR computation.
4. CPU sequence:

```text
# Write coefficients
for i in 0..TAPS-1: write32(BASE + 0x10 + 4*i, coeff[i])

# Write input sample
write32(BASE + 0x80, sample)

# Start FIR
write32(BASE + 0x00, 0x1)

# Poll until done
while((read32(BASE + 0x04) & 0x2) == 0)

# Read output
y = read32(BASE + 0x08)

# Clear start
write32(BASE + 0x00, 0x0)
```

---

## Reset Behavior

* Core reset (`rst_n` low) clears:

  * Shift registers
  * Coefficient memory (optional, may retain previous values if controlled externally)
  * Accumulator = 0
  * Status/DONE = 0

* Wrapper reset clears:

  * CTRL = 0
  * STATUS = 0
  * IN\_FIFO empty

---

## Optional Enhancements

* IRQ on DONE to reduce CPU polling.
* Streaming mode with full input FIFO + DMA.
* Pipelined MAC for 1-cycle throughput.
* FFT-based frequency-domain acceleration for long filters.
* Parallel MACs for higher throughput.

---

## FPGA / ASIC Integration Notes

* OpenLane/Sky130:

  * Use `fir_wb` as memory-mapped peripheral in OpenFrame user project area.
  * Ensure top-level signal widths match CPU bus (32-bit).
  * Add timing constraints in SDC for FIR clock domain.

* FPGA:

  * Connect `fir_wb` to Wishbone interconnect.
  * Optional: add FIFO for streaming input.
  * Run `verilog/dv/fir_test/Makefile` to verify simulation.

---

## Verification Strategy

* **Self-checking TB (`fir_tb.v`)**:

  * Programs coefficients
  * Feeds input samples
  * Computes software golden MAC
  * Compares output to golden reference
  * Logs PASS/FAIL

* **Observables**:

  * `clk`, `rst_n`, `start`, `sample_in`, `acc`, `sample_out`, `done`

* **Waveform dump**: `fir_tb.vcd` (GTKWave).

* **Corner cases**:

  * Max/min signed input for saturation
  * Coefficient changes mid-run
  * Reset during RUN

---

## File-Level Summary

| File            | Role                                                       |
| --------------- | ---------------------------------------------------------- |
| `fir_core.v`    | FIR datapath (shift register + MAC + accumulator)          |
| `fir_wb.v`      | Wishbone-Lite wrapper, memory-mapped interface             |
| `fifo_sync.v`   | Optional FIFO for input streaming and clock crossing       |
| `fir_tb.v`      | Self-checking testbench, software golden model             |
| `fir_test.c`    | Firmware example for Microwatt CPU                         |
| `Makefile` (sw) | Builds firmware using cross-toolchain for Microwatt        |
| `DESIGN.md`     | This document                                              |
| `README.md`     | High-level overview, motivation, integration, architecture |

---

## Notes / Future Roadmap

* Add IRQ for DONE
* Streaming / DMA interface
* Pipelined MAC for one-cycle throughput
* FFT-based convolution for large filters
* Formal verification of FSM and arithmetic
* Full OpenLane flow for Sky130
* Timing diagrams and latency charts

---
