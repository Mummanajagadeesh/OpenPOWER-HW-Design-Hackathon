// verilog/rtl/accel/fir_accel.sv
// Simple streaming FIR accelerator (parameterizable taps) with saturation
// - start: pulse to start processing one input sample
// - sample_in: input sample (signed 32-bit)
// - sample_out: output sample (signed 32-bit, saturated)
// - done: asserted one cycle when result ready

module fir_accel #(
  parameter integer TAPS  = 8,
  parameter integer WIDTH = 32
)(
  input  logic                     clk,
  input  logic                     rst_n,

  // control/data
  input  logic                     start,
  input  logic signed [WIDTH-1:0]  sample_in,
  output logic signed [WIDTH-1:0]  sample_out,
  output logic                     done,

  // coefficient write port (simple register write interface)
  input  logic                     coeff_wr_en,
  input  logic [$clog2(TAPS)-1:0]  coeff_wr_addr,
  input  logic signed [WIDTH-1:0]  coeff_wr_data
);

  // coefficient memory & sample shift regs
  logic signed [WIDTH-1:0] coeffs [0:TAPS-1];
  logic signed [WIDTH-1:0] regs   [0:TAPS-1];

  typedef enum logic [1:0] {IDLE=2'b00, RUN=2'b01, DONE_S=2'b10} state_t;
  state_t state;
  integer i;
  integer cycle_cnt;
  logic signed [2*WIDTH-1:0] acc;
  logic signed [WIDTH-1:0] sat_out;

  // main FSM
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (i=0;i<TAPS;i++) begin
        coeffs[i] <= '0;
        regs[i]   <= '0;
      end
      state       <= IDLE;
      cycle_cnt   <= 0;
      acc         <= '0;
      sample_out  <= '0;
      done        <= 0;
    end else begin
      // coefficient write
      if (coeff_wr_en) begin
        coeffs[coeff_wr_addr] <= coeff_wr_data;
      end

      case (state)
        IDLE: begin
          done <= 0;
          acc  <= '0;
          cycle_cnt <= 0;
          if (start) begin
            // shift in new sample
            for (i=TAPS-1;i>0;i--) regs[i] <= regs[i-1];
            regs[0] <= sample_in;
            acc     <= '0;
            state   <= RUN;
          end
        end

        RUN: begin
          acc <= acc + (regs[cycle_cnt] * coeffs[cycle_cnt]);
          cycle_cnt <= cycle_cnt + 1;
          if (cycle_cnt == TAPS-1) begin
            // saturate result
            if (acc > $signed({1'b0, {(WIDTH-1){1'b1}}}))
              sat_out = {1'b0, {(WIDTH-1){1'b1}}}; // max positive
            else if (acc < $signed({1'b1, {(WIDTH-1){1'b0}}}))
              sat_out = {1'b1, {(WIDTH-1){1'b0}}}; // max negative
            else
              sat_out = acc[WIDTH-1:0];

            sample_out <= sat_out;
            done       <= 1;
            state      <= DONE_S;
          end
        end

        DONE_S: begin
          // wait for next start
          if (!start) begin
            done  <= 0;
            state <= IDLE;
          end
        end

        default: state <= IDLE;
      endcase
    end
  end

endmodule
