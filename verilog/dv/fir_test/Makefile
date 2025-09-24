VERILOG_SOURCES = ../../rtl/fir_accel/fir_wb.v ../../rtl/fir_accel/fifo_sync.v ../../rtl/fir_accel/fir_core.v fir_tb.v
VVP_OUT = fir_tb.vvp
IVERILOG = iverilog
VVP = vvp
VCD = fir_test.vcd

.PHONY: all sim clean wave

all: sim

sim: $(VVP_OUT)
	@echo "Running FIR accelerator simulation..."
	$(VVP) $(VVP_OUT)

$(VVP_OUT): $(VERILOG_SOURCES)
	$(IVERILOG) -o $(VVP_OUT) $(VERILOG_SOURCES)

wave: sim
	@if [ -f $(VCD) ]; then \
		echo "Opening GTKWave..."; \
		gtkwave $(VCD) & \
	else \
		echo "No $(VCD) found"; \
	fi

clean:
	rm -f $(VVP_OUT) $(VCD) *.vcd *.vvp *.vpi *.log
