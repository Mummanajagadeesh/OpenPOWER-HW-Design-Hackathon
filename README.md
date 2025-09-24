
# FIR Accelerator — Microwatt/OpenFrame Hackathon Submission

![block diagram](assets/image.png)

---

## Executive Summary / Motivation

Digital signal processing (DSP) is pervasive in modern embedded systems: audio processing, sensor fusion, communications, motor control, and IoT edge nodes all rely on efficient linear filtering. Among linear filters, **FIR (Finite Impulse Response) filters** are widely used due to their predictable, stable, and linear-phase behavior.

In software, FIR filters are **CPU-intensive**, requiring repeated multiply–accumulate (MAC) operations. For a filter with `N` taps, each new sample requires `N` multiplications and `N-1` additions. On a low-power CPU like **Microwatt**, this can dominate execution cycles and increase energy consumption.

By implementing a **dedicated FIR accelerator**:

* CPU load is dramatically reduced.
* Latency is deterministic (exact cycles per output).
* Energy efficiency improves for real-time DSP workloads.
* Small area is achievable through a **sequential MAC design**, which reuses a single multiplier across taps.

This project implements a **parameterizable FIR accelerator** in Verilog, along with a **Wishbone-Lite memory-mapped wrapper** for integration with Microwatt/OpenFrame. The design supports ASIC (Sky130/OpenLane) and FPGA flows.

---

## Why FIR? Why Hardware Acceleration?

### FIR Filters

The FIR filter output is computed as:

$$
y[n] = \sum_{i=0}^{N-1} h[i] \cdot x[n-i]
$$

* `N` = number of taps (filter length).
* `h[i]` = filter coefficients.
* `x[n-i]` = previous input samples (delay line).

**Advantages of FIR filters:**

1. **Linear phase possible** — critical for audio and control systems.
2. **Always stable** — no feedback loops.
3. **Hardware-friendly arithmetic** — MAC operations can be pipelined or sequentially reused.

### Software vs Hardware

* CPU-based FIR: high instruction count, variable latency, higher energy per sample.
* Hardware FIR: deterministic, low-power, single-cycle multiply-add per tap.
* Sequential MAC: area-efficient (single multiplier), deterministic throughput of one output per `TAPS` cycles.

### Use Cases for Edge/SoC Systems

* Real-time audio filters (equalization, noise suppression).
* Sensor pre-processing (IMU, accelerometer, gyroscope).
* Control systems (PID filter pre-processing, feedback loops).
* Communications (matched filters, pulse shaping).

**Key point:** A small, efficient FIR core offloads the CPU, freeing cycles for higher-level tasks or enabling lower clock frequency for power savings.

---

## Project Directory & File Overview

```
.
├── assets/
│   └── image.png                  # Block diagram
├── docs/
│   └── DESIGN.md                  # Memory map & integration notes
├── sw/
│   ├── fir_test.c                 # Microwatt firmware example
│   └── Makefile                   # Build firmware with cross-toolchain
├── verilog/
│   ├── rtl/fir_accel/
│   │   ├── fir_core.v             # FIR datapath
│   │   ├── fir_wb.v               # Wishbone-Lite wrapper
│   │   └── fifo_sync.v            # Optional FIFO for input streaming
│   └── dv/fir_test/
│       ├── fir_tb.v               # Self-checking testbench
│       └── Makefile               # Run simulation (Icarus Verilog)
└── README.md
```

---

### File Descriptions

#### `verilog/rtl/fir_accel/fir_core.v` — FIR Datapath

* **Parameterizable**: supports configurable `DATA_W`, `COEF_W`, `ACC_W`, `MAX_TAPS`.

* **Ports**: `clk`, `rst_n`, `start_proc`, `in_sample`, `in_valid`, `out_sample`, `out_valid`.

* **Internal components**:

  * Shift-register array: stores last `MAX_TAPS` input samples.
  * Coefficient memory: stores `MAX_TAPS` coefficients.
  * Sequential MAC accumulator (`acc`).
  * FSM: `IDLE → RUN → DONE`.

* **Behavior**:

  * Accepts input when `in_valid` is high.
  * Computes output sequentially over `TAPS` cycles.
  * Saturates/truncates result to `DATA_W`.
  * Signals `out_valid` upon computation completion.

**Design notes**:

* Sequential MAC = area-efficient.
* Latency = `TAPS` cycles + FSM overhead.
* Could be extended to pipelined/parallel MACs for higher throughput.

---

#### `verilog/rtl/fir_accel/fir_wb.v` — Wishbone-Lite Wrapper

* Maps FIR core to **memory-mapped registers** accessible by CPU.

* **Registers**:

  * `0x00` — CTRL (bit0 = START)
  * `0x04` — STATUS (bit1 = OUT\_VALID)
  * `0x08` — DATAOUT (read FIR output)
  * `0x80` — IN\_FIFO (write input sample)
  * `0x10 + 4*i` — COEFF\[i]

* Generates one-cycle `coef_wr_en` pulses for FIR core.

* Single-cycle `wb_ack_o` per transaction.

**Software constraints**:

* Clear START after reading output.
* Coefficient writes during RUN produce undefined results — ensure proper sequence in firmware.

---

#### `verilog/rtl/fir_accel/fifo_sync.v` — FIFO

* Optional input queuing for **streaming data**.
* Supports clock domain crossing if needed.
* Depth configurable.

---

#### `verilog/dv/fir_test/fir_tb.v` — Testbench

* Self-checking: drives **core and wrapper**.

* Tasks:

  * `wb_write()`, `wb_read()` for Wishbone transactions.
  * `golden()` for reference MAC computation.
  * Handles latency due to sequential MAC.

* **Test cases**:

  * Deterministic sequences (moving average, all ones).
  * Randomized coefficients/input.
  * Saturation edge cases.

* Waveform output: `fir_tb.vcd`.

* Compile/run (Icarus Verilog):

```bash
make -C verilog/dv/fir_test
```

---

#### `sw/fir_test.c` — Firmware Example

* Demonstrates **CPU → FIR interaction** via memory-mapped registers:

```c
#define FIR_BASE 0x40000000

// Write coefficients
for(int i=0;i<TAPS;i++) write32(FIR_BASE+0x10+4*i, coeff[i]);

// Write input sample
write32(FIR_BASE+0x80, sample);

// Start FIR
write32(FIR_BASE+0x00, 0x1);

// Poll until done
while((read32(FIR_BASE+0x04) & 0x2) == 0);

// Read output
int out = read32(FIR_BASE+0x08);

// Clear START
write32(FIR_BASE+0x00, 0x0);
```

* Important: **clear START** after each run; core holds DONE until START is low.

---

#### `sw/Makefile`

* Builds firmware for Microwatt with appropriate cross-toolchain:

```Makefile
CC = powerpc-linux-gnu-gcc
CFLAGS = -O2 -Wall
TARGET = fir_test

all: $(TARGET)

$(TARGET): fir_test.c
    $(CC) $(CFLAGS) -o $(TARGET) fir_test.c

clean:
    rm -f $(TARGET) *.o
```

* Usage:

```bash
make -C sw/
./fir_test   # run on Microwatt simulation or hardware
```

---

#### `docs/DESIGN.md`

* **Memory map**: base addresses, register offsets, bitfields.
* **Integration**: OpenFrame/Microwatt instructions.
* **Placeholders**: timing diagrams, SDC constraints, FPGA/ASIC integration steps.

---

### High-Level Architecture

```
[Microwatt CPU] --> [Wishbone-Lite Wrapper] --> [FIR Core]
                             |                     |
                             |                     --> Shift register / MAC
                             |
                             --> FIFO (optional input queuing)
```

**Notes**:

* Sequential MAC = area-efficient.
* Latency = `TAPS` cycles per sample.
* Wrapper decouples CPU interface from FIR core.

---

### Operational Flow

1. Write coefficients (`COEFF[i]`).
2. Write input sample (`DATAIN`).
3. Set START (`CTRL.START = 1`).
4. FIR core computes sequential MAC over `TAPS` cycles.
5. `OUT_VALID` is asserted.
6. CPU reads output (`DATAOUT`).
7. Clear START (`CTRL.START = 0`).
8. Repeat for next sample.

---

### Latency & Timing Considerations

* Sequential MAC: one multiply-add per cycle.
* Deterministic latency: `TAPS + FSM overhead`.
* Throughput: one output per `TAPS` cycles.
* Future optimizations:

  * Pipelined MAC or multiple multipliers for 1-cycle throughput.
  * Streaming/DMA mode via FIFO.
  * FFT-based convolution for long filters.

---

### Verification Strategy

* Golden reference in TB.
* Deterministic & random tests.
* Edge cases: saturation, coefficient change mid-run.
* Observables: `out_sample`, `out_valid`.
* Waveform: `fir_tb.vcd`.

---

### Roadmap / Future Extensions

* IRQ / event-driven operation.
* Streaming/DMA support for continuous filtering.
* Parallel or pipelined MACs.
* FFT-based FIR acceleration for long filters.
* Formal verification for FSM and arithmetic correctness.
* OpenLane Sky130 ASIC flow integration.
* Expanded documentation: diagrams, timing charts, firmware sequences.

---
